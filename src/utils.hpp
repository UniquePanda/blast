#pragma once

#include <cstddef>
#include <iostream>
#include <string>
#include <optional>

enum class TokenType {
    unknown, line_break,
    let, reassign, ident, int_lit, dbl_lit, str_lit, bool_lit,
    built_in_func, if_, elseif, else_,
    eq, open_paren, close_paren, open_curly, close_curly, semi, quot,
    plus, minus, star, slash
};

struct Token {
    TokenType type;
    std::optional<std::string> value {};
};

std::string negUnOperatorCondensed(std::vector<Token> unaryOperators) {;
    int minusCount = 0;
    for (const Token unOperator : unaryOperators) {
        if (unOperator.type == TokenType::minus) {
            minusCount++;
        }
    }

    return minusCount % 2 == 0 ? "" : "-";
}

std::string unOpSymbol(TokenType unaryOperatorType) {
    switch (unaryOperatorType) {
        case TokenType::plus:
            return "+";

        case TokenType::minus:
            return "-";

        default:
            return "";
    }
}

std::string tokenTypeName(TokenType tokenType) {
    switch (tokenType) {
        case TokenType::unknown:
            return "Unknown";
        case TokenType::line_break:
            return "Line Break";
        case TokenType::let:
            return "Let";
        case TokenType::reassign:
            return "Reassign";
        case TokenType::ident:
            return "Identifier";
        case TokenType::int_lit:
            return "Integer";
        case TokenType::dbl_lit:
            return "Double";
        case TokenType::str_lit:
            return "String";
        case TokenType::bool_lit:
            return "Boolean";
        case TokenType::built_in_func:
            return "Built-in Function";
        case TokenType::if_:
            return "If";
        case TokenType::elseif:
            return "Else-If";
        case TokenType::else_:
            return "Else";
        case TokenType::eq:
            return "Equals";
        case TokenType::open_paren:
            return "Opening Parenthesis";
        case TokenType::close_paren:
            return "Closing Parenthesis";
        case TokenType::open_curly:
            return "Opening Curly Brace";
        case TokenType::close_curly:
            return "Closing Curly Brace";
        case TokenType::semi:
            return "Semicolon";
        case TokenType::quot:
            return "Quotation Mark";
        case TokenType::plus:
            return "Plus";
        case TokenType::minus:
            return "Minus";
        case TokenType::star:
            return "Star";
        case TokenType::slash:
            return "Slash";
    }

    return "Unknown";
}

void fail(const std::string& msg, const size_t& lineNumber) {
    std::cerr << "Line " << lineNumber << ": " << msg << std::endl;
    exit(EXIT_FAILURE);
}

void failUnexpectedEscape(const char& unknownEscapedChar, const size_t& lineNumber) {
    fail("Unexpected escape sequence: \\" + std::string(1, unknownEscapedChar), lineNumber);
}

void failUnknownChar(const char& unknownChar, const size_t& lineNumber) {
    fail("Unknown character: " + std::string(1, unknownChar), lineNumber);
}

void failUnexpectedNonDigit(const char&  unexpectedChar, const size_t& lineNumber) {
    fail("Unexpected non-digit character" + (unexpectedChar != ' ' ? ": " + std::string(1, unexpectedChar) : ""), lineNumber);
}

void failUnexpectedEOF(const size_t& lineNumber) {
        fail("Unexpected end of file", lineNumber);
}

void failInvalidStmt(const size_t& lineNumber) {
    fail("Invalid statement", lineNumber);
}

void failInvalidReassignment(const std::string& expectedType, const size_t& lineNumber) {
    fail("Invalid reassignment. Expected data type: " + expectedType, lineNumber);
}

void failInvalidScope(const size_t& lineNumber) {
    fail("Invalid scope", lineNumber);
}

void failInvalidExpr(const std::string& detail, const size_t& lineNumber) {
    fail("Invalid expression: " + detail, lineNumber);
}

void failMissingExpr(const std::string& detail, const size_t& lineNumber) {
    fail("Missing expression: " + detail, lineNumber);
}

void failMissingIdent(const std::string& detail, const size_t& lineNumber) {
    fail("Missing identifier: " + detail, lineNumber);
}

void failMissingOperator(const std::string& detail, const size_t& lineNumber) {
    fail("Missing operator: " + detail, lineNumber);
}

void failMissingStmt(const std::string& stmtName, const size_t& lineNumber) {
    fail("Missing statement" + (stmtName == "" ? "" : ": " + stmtName), lineNumber);
}

void failUnexpectedUnaryOperator(const std::string& op, const size_t& lineNumber) {
    fail("Unexpected unary operator: " + op, lineNumber);
}

void failUnsupportedBinaryOperator(const size_t& lineNumber) {
    fail("Unsupported binary operator", lineNumber);
}

void failBinaryRHS(const size_t& lineNumber) {
    fail("Missing right hand side expression", lineNumber);
}

void failMissingOpenParen(const size_t& lineNumber) {
    fail("Missing opening parenthesis", lineNumber);
}

void failMissingClosingParen(const size_t& lineNumber) {
    fail("Missing closing parenthesis", lineNumber);
}

void failMissingOpenCurly(const size_t& lineNumber) {
    fail("Missing opening curly brace", lineNumber);
}

void failMissingClosingCurly(const size_t& lineNumber) {
    fail("Missing closing curly brace", lineNumber);
}

void failMissingQuot(const size_t& lineNumber) {
    fail("Missing quotation mark", lineNumber);
}

void failMissingSemi(const size_t& lineNumber) {
    fail("Missing semicolon", lineNumber);
}

void failAlreadyUsedIdentifier(const std::string& identName, const size_t& lineNumber) {
    fail("Identifier already used: " + identName, lineNumber);
}

void failUndeclaredIdentifier(const std::string& identName, const size_t& lineNumber) {
    fail("Undeclared identifier: " + identName, lineNumber);
}

void failUnknownBuiltInFunc(const std::string& funcName, const size_t& lineNumber) {
    fail("Unknown built in function: " + funcName, lineNumber);
}

void failUnknownDataType(const size_t& lineNumber) {
    fail("Unknown data type", lineNumber);
}

void failExpectedDifferentDataType(const std::string& expectedDataType, const size_t& lineNumber) {
    fail("Expected different data type. Expected: " + expectedDataType, lineNumber);
}

void failUnexpectedDataType(const std::string& unexpectedDataType, const size_t& lineNumber) {
    fail("Encountered unexpected data type: " + unexpectedDataType, lineNumber);
}
