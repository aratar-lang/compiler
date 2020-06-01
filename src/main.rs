use std::env;
use std::fs::File;
use std::process;
use std::io::prelude::*;
use std::io::BufReader;

enum Token {
    // An identifier
    Ident(),
    // A number
    Number(),
}

// An iterator over lexemes in a character stream.
struct LexicalAnalyzer<R: Read> {
    reader: R,
}

impl<R: Read> LexicalAnalyzer<R> {
    fn new(reader: R) -> Self {
        LexicalAnalyzer {
            reader,
        }
    }
}

impl<R: Read> Iterator for LexicalAnalyzer<R> {
    type Item = Token;

    fn next(&mut self) -> Option<Self::Item> {
        /*while let Some() =  {
        }*/
        
        None
    }
}

struct CharStream<R: Read> {
    reader: R,
}

impl<R: Read> CharStream<R> {
    fn new(reader: R) -> Self {
        CharStream {
            reader,
        }
    }
}

impl<R: Read> Iterator for CharStream<R> {
    type Item = char;

    fn next(&mut self) -> Option<Self::Item> {
        let mut c = [0u8; 4];
        let mut ch = None;
        for i in 0..4 {
            if let Err(_) = self.reader.read_exact(&mut c[i..i+1]) {
                if i == 0 {
                    return None;
                }
                eprintln!("Unexpected end of file!");
                process::exit(1);
            }
            match std::str::from_utf8(&c[..i+1]) {
                Ok(c) => {
                    ch = c.chars().next();
                    break;
                }
                Err(_e) if _e.error_len().is_some() => {
                    eprintln!("Input file is not valid UTF-8!");
                    process::exit(1);
                }
                Err(_) => { /* Continue */ }
            }
        }
        let mut ch = ch.unwrap();

        Some(ch)        
    }
}

fn main() -> std::io::Result<()> {
    let mut args = env::args().skip(1);

    let filename = if let Some(arg) = args.next() {
        eprintln!("Compiling file: '{}'...",arg);
        arg
    } else {
        eprintln!("Need to pass file name!");
        process::exit(1);
    };

    let file = File::open(filename)?;
    let buf_reader = BufReader::new(file);

    let char_stream = CharStream::new(buf_reader);

    // LexicalAnalyzer::new(buf_reader);
    
    for c in char_stream {
        println!("{}", c);
    }
    
    Ok(())
}
