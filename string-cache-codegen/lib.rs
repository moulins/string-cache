// Copyright 2016 The Servo Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

#![recursion_limit = "128"]

extern crate phf_generator;
extern crate phf_shared;
extern crate string_cache_shared as shared;
#[macro_use] extern crate quote;

use std::collections::HashSet;
use std::fs::File;
use std::io::{self, Write, BufWriter};
use std::iter;
use std::path::Path;
use quote::Ident;

/// A builder for a static atom set and relevant macros
pub struct AtomType {
    base_type: Option<TypeInfo>,
    copiable_type: Option<TypeInfo>,
    atoms: HashSet<String>,
}

#[derive(Clone)]
struct TypeInfo {
    static_set_name: Ident,
    type_name: Ident,
    macro_name: Ident,
    path: Ident,
}

fn mk_type_info(path: &str, macro_name: &str) -> TypeInfo {
    assert!(macro_name.ends_with("!"));
    let type_name = if let Some(position) = path.rfind("::") {
        &path[position + "::".len() ..]
    } else {
        path
    }.to_owned();

    let static_set_name = Ident::from(format!("{}StaticSet", type_name));
    let type_name = Ident::from(type_name);

    let macro_name = &macro_name[..macro_name.len() - "!".len()];
    let macro_name = Ident::from(macro_name.to_owned());

    let path = Ident::from(path.to_owned());
    TypeInfo {
        static_set_name,
        type_name,
        macro_name,
        path
    }
}

impl AtomType {
    /// Constructs a new static atom set builder
    ///
    /// `path` is a path within a crate of the atom type that will be created.
    /// e.g. `"FooAtom"` at the crate root or `"foo::Atom"` if the generated code
    /// is included in a `foo` module.
    ///
    /// `macro_name` must end with `!`.
    ///
    /// For example, `AtomType::new("foo::FooAtom", "foo_atom!")` will generate:
    ///
    /// ```rust
    /// pub type FooAtom = ::string_cache::Atom<FooAtomStaticSet>;
    /// pub struct FooAtomStaticSet;
    /// impl ::string_cache::StaticAtomSet for FooAtomStaticSet {
    ///     // ...
    /// }
    /// #[macro_export]
    /// macro_rules foo_atom {
    ///    // Expands to: $crate::foo::FooAtom { … }
    /// }
    pub fn new(path: &str, macro_name: &str) -> Self {
        AtomType {
            base_type: Some(mk_type_info(path, macro_name)),
            copiable_type: None,
            atoms: HashSet::new(),
        }
    }

    /// Same as `AtomType::new`, but constructs the type `CopiableAtom` instead of `Atom`.
    /// This type implements `Copy`, but will leak when adding a new atom.
    /// 
    /// Requires the feature `copiable-atoms` of the `string-cache` crate.
    pub fn new_copiable(path: &str, macro_name: &str) -> Self {
        AtomType {
            base_type: None,
            copiable_type: Some(mk_type_info(path, macro_name)),
            atoms: HashSet::new(),
        }
    }

    /// Constructs the type `CopiableAtom` instead of `Atom`.
    /// This type implements `Copy`, but will leak when adding a new atom.
    /// 
    /// Requires the feature `copiable-atoms` of the `string-cache` crate.
    pub fn copiable(&mut self, path: &str, macro_name: &str) -> &mut Self {
        if let Some(_) = self.copiable_type {
            panic!("copiable type already defined")
        }
        self.copiable_type = Some(mk_type_info(path, macro_name));
        self
    }

    /// Adds an atom to the builder
    pub fn atom(&mut self, s: &str) -> &mut Self {
        self.atoms.insert(s.to_owned());
        self
    }

    /// Adds multiple atoms to the builder
    pub fn atoms<I>(&mut self, iter: I) -> &mut Self
    where I: IntoIterator, I::Item: AsRef<str> {
        self.atoms.extend(iter.into_iter().map(|s| s.as_ref().to_owned()));
        self
    }

    /// Write generated code to `destination`.
    pub fn write_to<W>(&mut self, mut destination: W) -> io::Result<()> where W: Write {
        destination.write_all(
            self.to_tokens()
            .as_str()
            // Insert some newlines to make the generated code slightly easier to read.
            .replace(" [ \"", "[\n\"")
            .replace("\" , ", "\",\n")
            .replace(" ( \"", "\n( \"")
            .replace("; ", ";\n")
            .as_bytes())
    }

    fn type_to_tokens(type_info: TypeInfo, atom_type: &str, static_set_name: quote::Ident,
            atoms: &[&str], data: &[quote::Hex<u64>]) -> quote::Tokens {
        let type_name = type_info.type_name;
        let path = iter::repeat(type_info.path);
        let macro_name = type_info.macro_name;
        let atom_type = Ident::from(atom_type);
        quote! {
            pub type #type_name = #atom_type<#static_set_name>;

            #[macro_export]
            macro_rules! #macro_name {
                #(
                    (#atoms) => {
                        $crate::#path {
                            unsafe_data: #data,
                            phantom: ::std::marker::PhantomData,
                        }
                    };
                )*
            }
        }
    }

    fn to_tokens(&mut self) -> quote::Tokens {
        // `impl Default for Atom` requires the empty string to be in the static set.
        // This also makes sure the set in non-empty,
        // which would cause divisions by zero in rust-phf.
        self.atoms.insert(String::new());

        let atoms: Vec<&str> = self.atoms.iter().map(|s| &**s).collect();
        let hash_state = phf_generator::generate_hash(&atoms);
        let phf_generator::HashState { key, disps, map } = hash_state;
        let atoms: Vec<&str> = map.iter().map(|&idx| atoms[idx]).collect();
        let empty_string_index = atoms.iter().position(|s| s.is_empty()).unwrap() as u32;
        let data: Vec<_> = (0..atoms.len()).map(|i| quote::Hex(shared::pack_static(i as u32))).collect();

        let hashes: Vec<u32> =
            atoms.iter().map(|string| {
                let hash = phf_shared::hash(string, key);
                ((hash >> 32) ^ hash) as u32
            }).collect();

        let static_set_name = & match (&self.base_type, &self.copiable_type) {
            (&Some(ref ty), _) => ty,
            (&None, &Some(ref ty)) => ty,
            _ => unreachable!()
        }.static_set_name.to_owned();

        let mut tokens = quote! {
            pub struct #static_set_name;
            impl ::string_cache::StaticAtomSet for #static_set_name {
                fn get() -> &'static ::string_cache::PhfStrSet {
                    static SET: ::string_cache::PhfStrSet = ::string_cache::PhfStrSet {
                        key: #key,
                        disps: &#disps,
                        atoms: &#atoms,
                        hashes: &#hashes
                    };
                    &SET
                }
                fn empty_string_index() -> u32 {
                    #empty_string_index
                }
            }
        };

        self.base_type.clone().map(|info| tokens.append(
            AtomType::type_to_tokens(info, "::string_cache::Atom", static_set_name.clone(), &atoms, &data)
        ));
        self.copiable_type.clone().map(|info| tokens.append(
            AtomType::type_to_tokens(info, "::string_cache::CopiableAtom", static_set_name.clone(), &atoms, &data)
        ));
        
        tokens
    }

    /// Create a new file at `path` and write generated code there.
    ///
    /// Typical usage:
    /// `.write_to_file(&Path::new(&env::var("OUT_DIR").unwrap()).join("foo_atom.rs"))`
    pub fn write_to_file(&mut self, path: &Path) -> io::Result<()> {
        self.write_to(BufWriter::new(try!(File::create(path))))
    }
}
