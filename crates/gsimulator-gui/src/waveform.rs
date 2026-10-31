//! Small expression evaluator for the built-in stimulus' custom waveform.
//!
//! The expression is written in `t` (simulated seconds); the stimulus drives
//! its pin high whenever the value is above `0.5`, so both plain arithmetic
//! (`sin(2*pi*2*t)`) and comparisons (`t > 0.1 && t < 0.3`) work.

use std::f64::consts;

/// A parsed stimulus expression.
#[derive(Clone, Debug)]
pub struct Expression {
    source: String,
    root: Node,
}

impl Expression {
    /// Parses `source`, reporting the first problem in Chinese.
    pub fn parse(source: &str) -> Result<Self, String> {
        let trimmed = source.trim();
        if trimmed.is_empty() {
            return Err("表达式为空".to_string());
        }
        let tokens = tokenize(trimmed)?;
        let mut parser = Parser { tokens, index: 0 };
        let root = parser.parse_or()?;
        if let Some(token) = parser.peek() {
            return Err(format!("表达式结尾有多余的 {}", token.describe()));
        }
        Ok(Expression {
            source: trimmed.to_string(),
            root,
        })
    }

    /// The text this expression was parsed from.
    pub fn source(&self) -> &str {
        &self.source
    }

    /// Evaluates the expression at simulated time `t` (seconds).
    pub fn eval(&self, t: f64) -> f64 {
        self.root.eval(t)
    }
}

#[derive(Clone, Debug)]
enum Node {
    Number(f64),
    Variable(Variable),
    Unary(UnaryOp, Box<Node>),
    Binary(BinaryOp, Box<Node>, Box<Node>),
    Call(Function, Vec<Node>),
}

impl Node {
    fn eval(&self, t: f64) -> f64 {
        match self {
            Node::Number(value) => *value,
            Node::Variable(variable) => variable.value(t),
            Node::Unary(op, inner) => {
                let value = inner.eval(t);
                match op {
                    UnaryOp::Neg => -value,
                    UnaryOp::Not => bool_value(value == 0.0),
                }
            }
            Node::Binary(op, left, right) => {
                let left = left.eval(t);
                match op {
                    // `&&` and `||` stop at the first decisive operand.
                    BinaryOp::And => bool_value(left != 0.0 && right.eval(t) != 0.0),
                    BinaryOp::Or => bool_value(left != 0.0 || right.eval(t) != 0.0),
                    _ => {
                        let right = right.eval(t);
                        match op {
                            BinaryOp::Add => left + right,
                            BinaryOp::Sub => left - right,
                            BinaryOp::Mul => left * right,
                            BinaryOp::Div => left / right,
                            BinaryOp::Rem => left % right,
                            BinaryOp::Pow => left.powf(right),
                            BinaryOp::Lt => bool_value(left < right),
                            BinaryOp::Le => bool_value(left <= right),
                            BinaryOp::Gt => bool_value(left > right),
                            BinaryOp::Ge => bool_value(left >= right),
                            BinaryOp::Eq => bool_value((left - right).abs() < 1e-9),
                            BinaryOp::Ne => bool_value((left - right).abs() >= 1e-9),
                            BinaryOp::And | BinaryOp::Or => unreachable!(),
                        }
                    }
                }
            }
            Node::Call(function, args) => {
                let values: Vec<f64> = args.iter().map(|node| node.eval(t)).collect();
                function.apply(&values)
            }
        }
    }
}

fn bool_value(value: bool) -> f64 {
    if value {
        1.0
    } else {
        0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Variable {
    T,
    Ms,
    Us,
    Pi,
    Tau,
    E,
}

impl Variable {
    fn from_name(name: &str) -> Option<Self> {
        match name {
            "t" => Some(Variable::T),
            "ms" => Some(Variable::Ms),
            "us" | "µs" => Some(Variable::Us),
            "pi" => Some(Variable::Pi),
            "tau" => Some(Variable::Tau),
            "e" => Some(Variable::E),
            _ => None,
        }
    }

    fn value(self, t: f64) -> f64 {
        match self {
            Variable::T => t,
            Variable::Ms => t * 1e3,
            Variable::Us => t * 1e6,
            Variable::Pi => consts::PI,
            Variable::Tau => consts::TAU,
            Variable::E => consts::E,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum UnaryOp {
    Neg,
    Not,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum BinaryOp {
    Add,
    Sub,
    Mul,
    Div,
    Rem,
    Pow,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    And,
    Or,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Function {
    Sin,
    Cos,
    Tan,
    Asin,
    Acos,
    Atan,
    Sqrt,
    Abs,
    Floor,
    Ceil,
    Round,
    Fract,
    Min,
    Max,
    Pow,
    Clamp,
    Step,
}

impl Function {
    fn from_name(name: &str) -> Option<Self> {
        match name {
            "sin" => Some(Function::Sin),
            "cos" => Some(Function::Cos),
            "tan" => Some(Function::Tan),
            "asin" => Some(Function::Asin),
            "acos" => Some(Function::Acos),
            "atan" => Some(Function::Atan),
            "sqrt" => Some(Function::Sqrt),
            "abs" => Some(Function::Abs),
            "floor" => Some(Function::Floor),
            "ceil" => Some(Function::Ceil),
            "round" => Some(Function::Round),
            "fract" => Some(Function::Fract),
            "min" => Some(Function::Min),
            "max" => Some(Function::Max),
            "pow" => Some(Function::Pow),
            "clamp" => Some(Function::Clamp),
            "step" => Some(Function::Step),
            _ => None,
        }
    }

    fn arity(self) -> usize {
        match self {
            Function::Min | Function::Max | Function::Pow => 2,
            Function::Clamp => 3,
            _ => 1,
        }
    }

    fn apply(self, args: &[f64]) -> f64 {
        match (self, args) {
            (Function::Sin, [x]) => x.sin(),
            (Function::Cos, [x]) => x.cos(),
            (Function::Tan, [x]) => x.tan(),
            (Function::Asin, [x]) => x.asin(),
            (Function::Acos, [x]) => x.acos(),
            (Function::Atan, [x]) => x.atan(),
            (Function::Sqrt, [x]) => x.sqrt(),
            (Function::Abs, [x]) => x.abs(),
            (Function::Floor, [x]) => x.floor(),
            (Function::Ceil, [x]) => x.ceil(),
            (Function::Round, [x]) => x.round(),
            (Function::Fract, [x]) => x - x.floor(),
            (Function::Min, [a, b]) => a.min(*b),
            (Function::Max, [a, b]) => a.max(*b),
            (Function::Pow, [a, b]) => a.powf(*b),
            (Function::Clamp, [x, lo, hi]) => {
                if x < lo {
                    *lo
                } else if x > hi {
                    *hi
                } else {
                    *x
                }
            }
            (Function::Step, [x]) => bool_value(*x > 0.0),
            _ => f64::NAN,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Number(f64),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    Percent,
    Caret,
    LParen,
    RParen,
    Comma,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    And,
    Or,
    Not,
}

impl Token {
    fn describe(&self) -> String {
        match self {
            Token::Number(value) => format!("数字 {value}"),
            Token::Ident(name) => format!("标识符 {name:?}"),
            Token::Plus => "'+'".to_string(),
            Token::Minus => "'-'".to_string(),
            Token::Star => "'*'".to_string(),
            Token::Slash => "'/'".to_string(),
            Token::Percent => "'%'".to_string(),
            Token::Caret => "'^'".to_string(),
            Token::LParen => "'('".to_string(),
            Token::RParen => "')'".to_string(),
            Token::Comma => "','".to_string(),
            Token::Lt => "'<'".to_string(),
            Token::Le => "'<='".to_string(),
            Token::Gt => "'>'".to_string(),
            Token::Ge => "'>='".to_string(),
            Token::Eq => "'=='".to_string(),
            Token::Ne => "'!='".to_string(),
            Token::And => "'&&'".to_string(),
            Token::Or => "'||'".to_string(),
            Token::Not => "'!'".to_string(),
        }
    }
}

fn tokenize(source: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let mut chars = source.char_indices().peekable();
    while let Some(&(index, current)) = chars.peek() {
        if current.is_whitespace() {
            chars.next();
            continue;
        }
        if current.is_ascii_digit() || current == '.' {
            let mut text = String::new();
            while let Some(&(_, digit)) = chars.peek() {
                if digit.is_ascii_digit() || digit == '.' {
                    text.push(digit);
                    chars.next();
                } else {
                    break;
                }
            }
            let value = text
                .parse::<f64>()
                .map_err(|_| format!("无效的数字 {text:?}（位置 {}）", index + 1))?;
            tokens.push(Token::Number(value));
            continue;
        }
        if current.is_ascii_alphabetic() || current == '_' {
            let mut name = String::new();
            while let Some(&(_, letter)) = chars.peek() {
                if letter.is_ascii_alphanumeric() || letter == '_' {
                    name.push(letter);
                    chars.next();
                } else {
                    break;
                }
            }
            tokens.push(Token::Ident(name));
            continue;
        }

        chars.next();
        let followed_by_equals = |chars: &mut std::iter::Peekable<std::str::CharIndices<'_>>| {
            if matches!(chars.peek(), Some(&(_, '='))) {
                chars.next();
                true
            } else {
                false
            }
        };
        let token = match current {
            '+' => Token::Plus,
            '-' => Token::Minus,
            '*' => Token::Star,
            '/' => Token::Slash,
            '%' => Token::Percent,
            '^' => Token::Caret,
            '(' => Token::LParen,
            ')' => Token::RParen,
            ',' => Token::Comma,
            '<' => {
                if followed_by_equals(&mut chars) {
                    Token::Le
                } else {
                    Token::Lt
                }
            }
            '>' => {
                if followed_by_equals(&mut chars) {
                    Token::Ge
                } else {
                    Token::Gt
                }
            }
            '=' => {
                if followed_by_equals(&mut chars) {
                    Token::Eq
                } else {
                    return Err(format!(
                        "单个 '=' 不是比较运算符，请用 '=='（位置 {}）",
                        index + 1
                    ));
                }
            }
            '!' => {
                if followed_by_equals(&mut chars) {
                    Token::Ne
                } else {
                    Token::Not
                }
            }
            '&' => {
                if matches!(chars.peek(), Some(&(_, '&'))) {
                    chars.next();
                    Token::And
                } else {
                    return Err(format!("'&' 需要写成 '&&'（位置 {}）", index + 1));
                }
            }
            '|' => {
                if matches!(chars.peek(), Some(&(_, '|'))) {
                    chars.next();
                    Token::Or
                } else {
                    return Err(format!("'|' 需要写成 '||'（位置 {}）", index + 1));
                }
            }
            other => {
                return Err(format!("无法识别的字符 {other:?}（位置 {}）", index + 1));
            }
        };
        tokens.push(token);
    }
    Ok(tokens)
}

struct Parser {
    tokens: Vec<Token>,
    index: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.index)
    }

    fn next(&mut self) -> Option<Token> {
        let token = self.tokens.get(self.index).cloned();
        if token.is_some() {
            self.index += 1;
        }
        token
    }

    fn eat(&mut self, expected: &Token) -> bool {
        if self.peek() == Some(expected) {
            self.index += 1;
            true
        } else {
            false
        }
    }

    fn expect(&mut self, expected: &Token) -> Result<(), String> {
        if self.eat(expected) {
            Ok(())
        } else {
            Err(format!("这里需要 {}", expected.describe()))
        }
    }

    fn parse_or(&mut self) -> Result<Node, String> {
        let mut node = self.parse_and()?;
        while self.eat(&Token::Or) {
            let right = self.parse_and()?;
            node = Node::Binary(BinaryOp::Or, Box::new(node), Box::new(right));
        }
        Ok(node)
    }

    fn parse_and(&mut self) -> Result<Node, String> {
        let mut node = self.parse_comparison()?;
        while self.eat(&Token::And) {
            let right = self.parse_comparison()?;
            node = Node::Binary(BinaryOp::And, Box::new(node), Box::new(right));
        }
        Ok(node)
    }

    fn parse_comparison(&mut self) -> Result<Node, String> {
        let mut node = self.parse_additive()?;
        loop {
            let op = match self.peek() {
                Some(Token::Lt) => BinaryOp::Lt,
                Some(Token::Le) => BinaryOp::Le,
                Some(Token::Gt) => BinaryOp::Gt,
                Some(Token::Ge) => BinaryOp::Ge,
                Some(Token::Eq) => BinaryOp::Eq,
                Some(Token::Ne) => BinaryOp::Ne,
                _ => break,
            };
            self.index += 1;
            let right = self.parse_additive()?;
            node = Node::Binary(op, Box::new(node), Box::new(right));
        }
        Ok(node)
    }

    fn parse_additive(&mut self) -> Result<Node, String> {
        let mut node = self.parse_multiplicative()?;
        loop {
            let op = match self.peek() {
                Some(Token::Plus) => BinaryOp::Add,
                Some(Token::Minus) => BinaryOp::Sub,
                _ => break,
            };
            self.index += 1;
            let right = self.parse_multiplicative()?;
            node = Node::Binary(op, Box::new(node), Box::new(right));
        }
        Ok(node)
    }

    fn parse_multiplicative(&mut self) -> Result<Node, String> {
        let mut node = self.parse_unary()?;
        loop {
            let op = match self.peek() {
                Some(Token::Star) => BinaryOp::Mul,
                Some(Token::Slash) => BinaryOp::Div,
                Some(Token::Percent) => BinaryOp::Rem,
                _ => break,
            };
            self.index += 1;
            let right = self.parse_unary()?;
            node = Node::Binary(op, Box::new(node), Box::new(right));
        }
        Ok(node)
    }

    fn parse_unary(&mut self) -> Result<Node, String> {
        match self.peek() {
            Some(Token::Minus) => {
                self.index += 1;
                Ok(Node::Unary(UnaryOp::Neg, Box::new(self.parse_unary()?)))
            }
            Some(Token::Plus) => {
                self.index += 1;
                self.parse_unary()
            }
            Some(Token::Not) => {
                self.index += 1;
                Ok(Node::Unary(UnaryOp::Not, Box::new(self.parse_unary()?)))
            }
            _ => self.parse_power(),
        }
    }

    fn parse_power(&mut self) -> Result<Node, String> {
        let base = self.parse_primary()?;
        if self.eat(&Token::Caret) {
            // Right associative, and `-2^2` means `-(2^2)`.
            let exponent = self.parse_unary()?;
            return Ok(Node::Binary(
                BinaryOp::Pow,
                Box::new(base),
                Box::new(exponent),
            ));
        }
        Ok(base)
    }

    fn parse_primary(&mut self) -> Result<Node, String> {
        match self.next() {
            Some(Token::Number(value)) => Ok(Node::Number(value)),
            Some(Token::Ident(name)) => {
                if self.eat(&Token::LParen) {
                    let function =
                        Function::from_name(&name).ok_or_else(|| format!("未知的函数 {name}"))?;
                    let mut args = Vec::new();
                    if !self.eat(&Token::RParen) {
                        loop {
                            args.push(self.parse_or()?);
                            if self.eat(&Token::Comma) {
                                continue;
                            }
                            self.expect(&Token::RParen)?;
                            break;
                        }
                    }
                    let arity = function.arity();
                    if args.len() != arity {
                        return Err(format!(
                            "{name} 需要 {arity} 个参数，实际给了 {} 个",
                            args.len()
                        ));
                    }
                    Ok(Node::Call(function, args))
                } else {
                    let variable = Variable::from_name(&name)
                        .ok_or_else(|| format!("未知的变量 {name}（可用 t / ms / pi）"))?;
                    Ok(Node::Variable(variable))
                }
            }
            Some(Token::LParen) => {
                let node = self.parse_or()?;
                self.expect(&Token::RParen)?;
                Ok(node)
            }
            Some(token) => Err(format!("这里不能出现 {}", token.describe())),
            None => Err("表达式不完整".to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Expression;

    fn eval(source: &str, t: f64) -> f64 {
        Expression::parse(source)
            .expect("expression must parse")
            .eval(t)
    }

    #[test]
    fn arithmetic_follows_the_usual_precedence() {
        assert_eq!(eval("1 + 2 * 3", 0.0), 7.0);
        assert_eq!(eval("(1 + 2) * 3", 0.0), 9.0);
        assert_eq!(eval("2 ^ 3 ^ 2", 0.0), 512.0);
        assert_eq!(eval("-2 ^ 2", 0.0), -4.0);
        assert_eq!(eval("7 % 4", 0.0), 3.0);
    }

    #[test]
    fn variables_expose_simulated_time() {
        assert_eq!(eval("t", 0.25), 0.25);
        assert_eq!(eval("ms", 0.25), 250.0);
        assert_eq!(eval("us", 0.25), 250_000.0);
        assert!((eval("tau", 0.0) - std::f64::consts::TAU).abs() < 1e-12);
    }

    #[test]
    fn functions_and_comparisons_work() {
        assert_eq!(eval("min(3, 1 + 1)", 0.0), 2.0);
        assert_eq!(eval("max(3, pow(2, 2))", 0.0), 4.0);
        assert_eq!(eval("clamp(5, 0, 1)", 0.0), 1.0);
        assert_eq!(eval("step(1 - t)", 0.5), 1.0);
        assert_eq!(eval("step(-1)", 0.0), 0.0);
        assert_eq!(eval("t >= 0.1", 0.2), 1.0);
        assert_eq!(eval("t >= 0.1", 0.05), 0.0);
        assert_eq!(eval("t > 0 && t < 1", 0.5), 1.0);
        assert_eq!(eval("!(t > 0)", 0.5), 0.0);
        assert_eq!(eval("fract(2.75)", 0.0), 0.75);
        assert!((eval("sin(pi / 2)", 0.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn bad_expressions_report_a_readable_error() {
        for source in ["", "1 +", "sin(", "t = 1", "nope", "sin(1, 2)", "1 & 2"] {
            let error = Expression::parse(source).expect_err(source);
            assert!(!error.is_empty(), "{source:?} produced an empty error");
        }
    }

    #[test]
    fn parsed_expression_keeps_its_source() {
        let expression = Expression::parse(" sin(t) ").unwrap();
        assert_eq!(expression.source(), "sin(t)");
    }
}
