use std::env;
use std::fs::File;
use std::process;
use std::io::BufReader;

mod lexical_analyzer;
mod char_stream;
mod token;

use char_stream::CharStream;
use token::Token;
use lexical_analyzer::LexicalAnalyzer;

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

    let chars = CharStream::new(buf_reader);
    let lexa = LexicalAnalyzer::new(chars);

    for token in lexa {
        println!("{:?}", token);
    }

    Ok(())
}
