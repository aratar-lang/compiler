#[derive(Debug)]
pub(super) enum Token {
    // An ascii identifier
    Ident([u8; 32]),
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
