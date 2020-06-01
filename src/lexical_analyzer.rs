use std::process;
use std::iter::Peekable;
use std::io::prelude::*;

use crate::{CharStream, Token};

// An iterator over lexemes in a character stream.
pub(super) struct LexicalAnalyzer<R: Read> {
    chars: Peekable<CharStream<R>>,
    line: usize,
}

impl<R: Read> LexicalAnalyzer<R> {
    pub(super) fn new(chars: CharStream<R>) -> Self {
        LexicalAnalyzer {
            chars: chars.peekable(),
            line: 0,
        }
    }
    
    // Skip stuff until no more whitespace, comments, etc.
    fn skip(&mut self) {
        'skip: while let Some(ch) = self.chars.peek() {
            match ch {
                ' ' => { /* ignore spaces */ }
                '#' => {
                    // Ignore Comment
                    self.chars.next();
                    'rem_delim: while let Some(ch) = self.chars.peek() {
                        match ch {
                            '#' | '\n' => {
                                break 'rem_delim;
                            }
                            _ => { self.chars.next(); }
                        }
                    }
                }
                '\n' => {
                    self.line += 1;
                }
                _ => { break 'skip }
            }
            self.chars.next();
        }
    }
}

impl<R: Read> Iterator for LexicalAnalyzer<R> {
    type Item = Token;

    fn next(&mut self) -> Option<Self::Item> {
        // Skip stuff until no more spaces, comments, etc.
        self.skip();
        // Get first character
        let ch = if let Some(ch) = self.chars.next() {
            ch
        } else {
            return None;
        };
        match ch {
            '&' => {
                match self.chars.peek() {
                    Some(':') => {
                        self.chars.next();
                        return Some(Token::AndAssign)
                    },
                    Some('&') => {
                        self.chars.next();
                        return Some(Token::AndSet)
                    },
                    _ => return Some(Token::And),
                }
            }
/*
            a if a.is_ascii_alphabetic() => {
                
            }
            d if d.is_ascii_digit() => {
                
            }
            w if w.is_whitespace() => {
                eprintln!("Invalid whitespace on line {}, only spaces and \
                    newlines allowed.", self.line);
                process::exit(1);
            }*/
            c => {
                eprintln!("Unknown character on line {}: '{}'", self.line, c);
                process::exit(1);
            }
        }
    
        /*while let Some(ch) = self.chars.peek() {

        }*/
        
        None
    }
}
