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
            line: 1,
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
                                if *ch == '\n' {
                                    self.line += 1;
                                }
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

    // Parse a base 10 number that could have a fractional component.
    fn base_ten(&mut self, start: char) -> Token {
        let mut value = start as u128 - '0' as u128;
        while let Some(digit) = self.chars.peek() {
            match digit {
                d if d.is_ascii_digit() => {
                    value = value * 10 + (*d as u128 - '0' as u128);
                }
                '_' => { /* skip */ }
                '.' => break,
                _ => return Token::Int(value),
            }
            self.chars.next();
        }
        let mut frac = match self.chars.next() {
            Some(d) if d.is_ascii_digit() => {
                d as u128 - '0' as u128
            }
            _ => {
                eprintln!("Decimal points must be followed by a digit");
                eprintln!("See line {}", self.line);
                process::exit(1);
            }
        };
        while let Some(digit) = self.chars.peek() {
            match digit {
                d if d.is_ascii_digit() => {
                    frac = frac * 10 + (*d as u128 - '0' as u128);
                }
                '_' => { /* ignore */ }
                _ => break
            }
            self.chars.next();
        }
        Token::Dec(value, frac)
    }

    fn base_sixteen(&mut self, start: char) -> Token {
        let mut value = hexdigit(start);
        while let Some(digit) = self.chars.peek() {
            match digit {
                d if d.is_ascii_digit() => {
                    value = value * 16 + hexdigit(*d);
                }
                '_' => { /* skip */ }
                _ => break,
            }
            self.chars.next();
        }
        Token::Int(value)
    }
    
    fn base_two(&mut self, start: char) -> Token {
        let mut value = start as u128 - '0' as u128;
        while let Some(digit) = self.chars.peek() {
            match digit {
                '0' | '1' => {
                    value = value * 2 + (*digit as u128 - '0' as u128);
                }
                '_' => { /* skip */ }
                _ => break,
            }
            self.chars.next();
        }
        Token::Int(value)
    }
    
    fn ident(&mut self, start: char) -> Token {
        let mut ident = [0u8; 32];
        ident[0] = start as u8;
        let fail_uppercase = start.is_lowercase();
        let mut index = 1;
        while let Some(ch) = self.chars.peek() {
            match ch {
                c if c.is_ascii_alphabetic() || *c == '_' => {
                    if c.is_ascii_uppercase() && fail_uppercase {
                        eprintln!("Invalid identifier: forbidden \
                            lowerCamelCase on line {}!", self.line);
                        process::exit(1);
                    }
                    ident[index] = *c as u8;
                }
                _ => break,
            }
            self.chars.next();
            index += 1;
            if index == 32 {
                eprintln!("Identifier is too long on line {}", self.line);
                process::exit(1);
            }
        }
        Token::Ident(ident)
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
                        Some(Token::AndAssign)
                    },
                    Some('&') => {
                        self.chars.next();
                        if self.chars.peek() == Some(&':') {
                            self.chars.next();
                            Some(Token::AndSetAssign)
                        } else {
                            Some(Token::AndSet)
                        }
                    },
                    _ => Some(Token::And),
                }
            }
            '@' => Some(Token::Ref),
            d if d.is_ascii_digit() => Some(self.base_ten(d)),
            a if a.is_ascii_alphabetic() || a == '_' => {
                match self.chars.next() {
                    Some(c) if c.is_ascii_lowercase() || a == '_' => {
                        Some(self.ident(c))
                    }
                    Some(c) if c.is_ascii_alphanumeric() => {
                        match a {
                            'x' => {
                                Some(self.base_sixteen(c))
                            }
                            'f' => {
                                if c.is_ascii_digit() {
                                    Some(self.base_ten(c))
                                } else {
                                    eprintln!("Malformed Number!");
                                    process::exit(1);
                                }
                            }
                            'b' => {
                                if c == '1' || c == '0' {
                                    Some(self.base_two(c))
                                } else {
                                    eprintln!("Malformed Number!");
                                    process::exit(1);
                                }
                            }
                            _ => {
                                eprintln!("Unknown number format on line {}!", self.line);
                                process::exit(1);
                            }
                        }

                    }
                    None => {
                        let mut ident = [0; 32];
                        ident[0] = a as u8;
                        Some(Token::Ident(ident))
                    }
                    _ => {
                        eprintln!("Invalid character in identifier on line {}", self.line);
                        process::exit(1);
                    }
                }
            }
            w if w.is_whitespace() => {
                eprintln!("Invalid whitespace on line {}, only spaces and \
                    newlines allowed.", self.line);
                process::exit(1);
            }
            c => {
                eprintln!("Unknown character on line {}: '{}'", self.line, c);
                process::exit(1);
            }
        }
    }
}

/// Returns true if character could be part of a hexadecimal literal.
fn hexdigit(c: char) -> u128 {
    if c.is_ascii_digit() {
        c as u128 - '0' as u128
    } else if c >= 'A' && c <= 'F' {
        c as u128 - ('A' as u128 + 10)
    } else {
        eprintln!("Invalid hexadecimal: '{}'", c.escape_default());
        process::exit(1);
    }
}
