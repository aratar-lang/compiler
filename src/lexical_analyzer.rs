use std::io::prelude::*;

use crate::{CharStream, Token};

// An iterator over lexemes in a character stream.
pub(super) struct LexicalAnalyzer<R: Read> {
    chars: CharStream<R>,
}

impl<R: Read> LexicalAnalyzer<R> {
    pub(super) fn new(chars: CharStream<R>) -> Self {
        LexicalAnalyzer {
            chars,
        }
    }
}

impl<R: Read> Iterator for LexicalAnalyzer<R> {
    type Item = Token;

    fn next(&mut self) -> Option<Self::Item> {
        
        None
    }
}
