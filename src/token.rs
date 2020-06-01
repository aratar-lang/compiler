pub(super) struct TokenString(pub(super) [u8; 32]);

impl std::fmt::Debug for TokenString {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", std::str::from_utf8(&self.0).unwrap())
    }
}

#[derive(Debug)]
pub(super) enum Token {
    // An ascii identifier
    Ident(TokenString),
    // An integer
    Int(u128),
    // A decimal
    Dec(u128, u128),

    //
    Ref, // @

    // And tokens
    And, // &
    AndAssign, // &:
    AndSet, // &&
    AndSetAssign, // &&:
    
}
