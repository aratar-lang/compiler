#[derive(Debug)]
pub(super) enum Token {
    // An identifier
    Ident(),
    // A number
    Number(),
    
    // And tokens
    And,
    AndAssign,
    AndSet,
}
