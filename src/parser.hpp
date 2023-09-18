#pragma once

#include <variant>
#include <vector>
#include <optional>

#include "arenaAllocator.hpp"
#include "tokenizer.hpp"

struct ExprNode;

struct IntLitTermNode {
    Token int_lit;
};

struct DblLitTermNode {
    Token dbl_lit;
};

struct StrLitTermNode {
    Token str_lit;
};

struct BoolLitTermNode {
    Token bool_lit;
};

struct IdentTermNode {
    Token ident;
};

struct ParenTermNode {
    ExprNode* expr;
};

struct TermNode {
    std::variant<IdentTermNode*, IntLitTermNode*, DblLitTermNode*, StrLitTermNode*, BoolLitTermNode*, ParenTermNode*> var;
};

struct SumBinExprNode {
    ExprNode* lhs;
    ExprNode* rhs;
};

struct SubBinExprNode {
    ExprNode* lhs;
    ExprNode* rhs;
};

struct MulBinExprNode {
    ExprNode* lhs;
    ExprNode* rhs;
};

struct DivBinExprNode {
    ExprNode* lhs;
    ExprNode* rhs;
};

struct BinExprNode {
    std::variant<SumBinExprNode*, SubBinExprNode*, MulBinExprNode*, DivBinExprNode*> var;
};

struct ExprNode {
    std::variant<TermNode*, BinExprNode*> var;
};

struct StmtNode;

struct BuiltInFuncStmtNode {
    std::string funcName;
    ExprNode* expr;
};

struct LetStmtNode {
    Token ident;
    ExprNode* expr;
};

struct ScopeNode {
    std::vector<StmtNode*> stmts;
};

struct IfStmtNode {
    ExprNode* expr;
    ScopeNode* scope;
};

struct ElseIfStmtNode {
    ExprNode* expr;
    ScopeNode* scope;
};

struct ElseStmtNode {
    ScopeNode* scope;
};

struct StmtNode {
    std::variant<BuiltInFuncStmtNode*, LetStmtNode*, ScopeNode*, IfStmtNode*, ElseIfStmtNode*, ElseStmtNode*> var;
};

struct ProgNode {
    std::vector<StmtNode*> stmts;
};

class Parser {
public:
    Parser(std::vector<Token> tokens)
        : m_tokens(std::move(tokens)),
        m_allocator(1024 * 1024 * 4) // 4 MB
        {}

    std::optional<TermNode*> parseTerm() {
        m_lastStmt = {};

        if (!peek().has_value()) {
            return {};
        }

        if (peek().value().type == TokenType::int_lit) {
            auto intLitTerm = m_allocator.alloc<IntLitTermNode>();
            intLitTerm->int_lit = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = intLitTerm;
            return term;
        } else if (peek().value().type == TokenType::dbl_lit) {
            auto dblLitTerm = m_allocator.alloc<DblLitTermNode>();
            dblLitTerm->dbl_lit = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = dblLitTerm;
            return term;
        } else if (peek().value().type == TokenType::quot) {
            if (!peek(1).has_value() || peek(1).value().type != TokenType::str_lit) {
                failInvalidExpr();
            }

            if (!peek(2).has_value() || peek(2).value().type != TokenType::quot) {
                failMissingQuot();
            }

            // Opening quotation mark.
            consume();

            auto strLitTerm = m_allocator.alloc<StrLitTermNode>();
            strLitTerm->str_lit = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = strLitTerm;

            // Closing quotation mark.
            consume();

            return term;
        } else if (peek().value().type == TokenType::bool_lit) {
            auto boolLitTerm = m_allocator.alloc<BoolLitTermNode>();
            boolLitTerm->bool_lit = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = boolLitTerm;
            return term;
        } else if (peek().value().type == TokenType::ident) {
            auto identTerm = m_allocator.alloc<IdentTermNode>();
            identTerm->ident = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = identTerm;
            return term;
        } else if (peek().value().type == TokenType::open_paren) {
            // Opening prenthesis
            consume();

            auto expr = parseExpr();

            if (!expr.has_value()) {
                failMissingExpr();
            }

            if (!peek().has_value() || peek().value().type != TokenType::close_paren) {
                failMissingClosingParen();
            }
            // Closing prenthesis
            consume();

            auto parenTerm = m_allocator.alloc<ParenTermNode>();
            parenTerm->expr = expr.value();
            auto term = m_allocator.alloc<TermNode>();
            term->var = parenTerm;
            return term;
        } else {
            return {};
        }
    }

    template <typename T> T* parseSpecificBinExpr(ExprNode *lhsExpr, size_t precedence) {
        m_lastStmt = {};

        auto specificBinExpr = m_allocator.alloc<T>();
        specificBinExpr->lhs = lhsExpr;

        if (auto rhs = parseExpr(precedence)) {
            specificBinExpr->rhs = rhs.value();
            return specificBinExpr;
        } else {
            failBinaryRHS();
        }

        return {};
    }

    std::optional<BinExprNode*> parseBinExpr(TokenType tokenType, ExprNode *lhsExpr, size_t precedence) {
        m_lastStmt = {};

        consume();
        auto binExpr = m_allocator.alloc<BinExprNode>();

        switch (tokenType) {
            case TokenType::plus:
                binExpr->var = parseSpecificBinExpr<SumBinExprNode>(lhsExpr, precedence);
                break;
            case TokenType::minus:
                binExpr->var = parseSpecificBinExpr<SubBinExprNode>(lhsExpr, precedence);
                break;
            case TokenType::star:
                binExpr->var = parseSpecificBinExpr<MulBinExprNode>(lhsExpr, precedence);
                break;
            case TokenType::slash:
                binExpr->var = parseSpecificBinExpr<DivBinExprNode>(lhsExpr, precedence);
                break;
            default:
                failUnsupportedBinaryOperator();
        }

        return binExpr;
    }

    std::optional<ExprNode*> parseExpr(size_t precedence = 0) {
        m_lastStmt = {};

        if (!peek().has_value()) {
            return {};
        }

        if (precedence >= Token::MAX_PRECEDENCE) {
            if (auto term = parseTerm()) {
                auto expr = m_allocator.alloc<ExprNode>();
                expr->var = term.value();
                return expr;
            } else {
                return {};
            }
        }

        if (auto lhsExpr = parseExpr(precedence + 1)) {
            if (peek().value().precedence != precedence) {
                return lhsExpr;
            }

            if (auto binExpr = parseBinExpr(peek().value().type, lhsExpr.value(), precedence)) {
                auto expr = m_allocator.alloc<ExprNode>();
                expr->var = binExpr.value();
                return expr;
            }
        } else {
            return {};
        }

        return {};
    }

    std::optional<ScopeNode*> parseScope() {
        // Don't set m_lastStmt here, because scopes are often used between other statements which might depend on each other.
        if (!peek().has_value()) {
            return {};
        }

        if (peek().value().type != TokenType::open_curly) {
            failMissingOpenCurly();
        }

        if (!peek(1).has_value()) {
            failUnexpectedEOF();
        }

        // Open curly
        consume();

        auto scopeStmt = m_allocator.alloc<ScopeNode>();

        while (auto stmt = parseStmt()) {
            scopeStmt->stmts.push_back(stmt.value());
        }

        if (!peek().has_value()) {
            failUnexpectedEOF();
        }

        if (peek().value().type != TokenType::close_curly) {
            failMissingClosingCurly();
        }

        // Closing curly
        consume();

        return scopeStmt;
    }

    std::optional<StmtNode*> parseStmt() {
        if (!peek().has_value()) {
            return {};
        }

        if (peek().value().type == TokenType::built_in_func) {
            if (!peek(1).has_value()) {
                failUnexpectedEOF();
            }

            if (peek(1).value().type != TokenType::open_paren) {
                failMissingOpenParen();
            }

            auto builtInFuncStmt = m_allocator.alloc<BuiltInFuncStmtNode>();
            auto funcName = peek().value().value.value();
            builtInFuncStmt->funcName = funcName;

            // function name and opening parenthesis
            consume();
            consume();

            if (auto exprNode = parseExpr()) {
                builtInFuncStmt->expr = exprNode.value();
            } else {
                failInvalidExpr();
            }
            if (peek().has_value() && peek().value().type == TokenType::close_paren) {
                consume();
            } else {
                failMissingClosingParen();
            }

            consumeSemi();

            if (builtInFuncStmt->funcName == "exit" && peek().has_value()) {
                std::cerr << "Found code after exit()" << std::endl;
                exit(EXIT_FAILURE);
            }

            m_lastStmt = TokenType::built_in_func;

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = builtInFuncStmt;
            return stmt;
        } else if (peek().value().type == TokenType::let) {
            if (!peek(1).has_value()) {
                failUnexpectedEOF();
            }

            if (peek(1).value().type != TokenType::ident) {
                failMissingIdent();
            }

            if (peek(2).value().type != TokenType::eq) {
                failMissingOperator();
            }

            // let
            consume();

            // ident
            auto letStmt = m_allocator.alloc<LetStmtNode>();
            letStmt->ident = consume();

            // equal sign
            consume();

            if (auto expr = parseExpr()) {
                letStmt->expr = expr.value();
            } else {
                failInvalidExpr();
            }

            consumeSemi();

            m_lastStmt = TokenType::let;

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = letStmt;
            return stmt;
        } else if (peek().value().type == TokenType::open_curly) {
            // Don't set m_lastStmt here, because scopes are often used between other statements which might depend on each other.
            if (auto scopeStmt = parseScope()) {
                auto stmt = m_allocator.alloc<StmtNode>();
                stmt->var = scopeStmt.value();
                return stmt;
            } else {
                failInvalidScope();
            }
        } else if (peek().value().type == TokenType::if_) {
            if (!peek(1).has_value()) {
                failUnexpectedEOF();
            }

            if (peek(1).value().type != TokenType::open_paren) {
                failMissingOpenParen();
            }

            // if and open parenthesis
            consume();
            consume();

            auto ifStmt = m_allocator.alloc<IfStmtNode>();

            if (auto expr = parseExpr()) {
                ifStmt->expr = expr.value();
            } else {
                failMissingExpr();
            }

            if (peek().value().type != TokenType::close_paren) {
                failMissingClosingParen();
            }

            // Closing parenthesis
            consume();

            if (auto scopeStmt = parseScope()) {
                ifStmt->scope = scopeStmt.value();
            } else {
                failInvalidScope();
            }

            m_lastStmt = TokenType::if_;

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = ifStmt;
            return stmt;
        } else if (peek().value().type == TokenType::elseif) {
            if (m_lastStmt != TokenType::if_ && m_lastStmt != TokenType::elseif) {
                failMissingStmt("if");
            }

            if (!peek(1).has_value()) {
                failUnexpectedEOF();
            }

            if (peek(1).value().type != TokenType::open_paren) {
                failMissingOpenParen();
            }

            // elseif and open parenthesis
            consume();
            consume();

            auto elseIfStmt = m_allocator.alloc<ElseIfStmtNode>();

            if (auto expr = parseExpr()) {
                elseIfStmt->expr = expr.value();
            } else {
                failMissingExpr();
            }

            if (peek().value().type != TokenType::close_paren) {
                failMissingClosingParen();
            }

            // Closing parenthesis
            consume();

            if (auto scopeStmt = parseScope()) {
                elseIfStmt->scope = scopeStmt.value();
            } else {
                failInvalidScope();
            }

            m_lastStmt = TokenType::elseif;

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = elseIfStmt;
            return stmt;
        } else if (peek().value().type == TokenType::else_) {
            if (m_lastStmt != TokenType::if_ && m_lastStmt != TokenType::elseif) {
                failMissingStmt("if");
            }

            if (!peek(1).has_value()) {
                failUnexpectedEOF();
            }

            // else
            consume();

            auto elseStmt = m_allocator.alloc<ElseStmtNode>();

            if (auto scopeStmt = parseScope()) {
                elseStmt->scope = scopeStmt.value();
            } else {
                failInvalidScope();
            }

            m_lastStmt = TokenType::else_;

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = elseStmt;
            return stmt;
        } else {
            return {};
        }

        return {};
    }

    std::optional<ProgNode> parseProg() {
        ProgNode prog;
        while (peek().has_value()) {
            if (auto stmt = parseStmt()) {
                prog.stmts.push_back(stmt.value());
            } else {
                failInvalidStmt();
            }
        }

        std::cout << "      ## Parser used " << m_allocator.usedSizeInBytes() << " Bytes." << std::endl;

        return prog;
    }

private:
    [[nodiscard]] std::optional<Token> peek(int offset = 0) const {
        if (m_index + offset >= m_tokens.size()) {
            return {};
        } else {
            return m_tokens.at(m_index + offset);
        }
    }

    Token consume() {
        return m_tokens.at(m_index++);
    }

    void consumeSemi() {
        if (peek().has_value() && peek().value().type == TokenType::semi) {
            consume();
        } else {
            failMissingSemi();
        }
    }

    void fail(std::string msg) const {
        std::cerr << msg << std::endl;
        exit(EXIT_FAILURE); 
    }

    void failUnexpectedEOF() const {
        fail("Unexpected end of file");
    }

    void failInvalidStmt() const {
        fail("Invalid statement");
    }

    void failInvalidScope() const {
        fail("Invalid scope");
    }

    void failInvalidExpr() const {
        fail("Invalid expression");
    }

    void failMissingExpr() const {
        fail("Missing expression");
    }

    void failMissingIdent() const {
        fail("Missing identifier");
    }

    void failMissingOperator() const {
        fail("Missing operator");
    }

    void failMissingStmt(std::string stmtName = "") const {
        fail("Missing statement" + (stmtName == "" ? "" : ": " + stmtName));
    }

    void failUnsupportedBinaryOperator() const {
        fail("Unsupported binary operator");
    }

    void failBinaryRHS() const {
        fail("Missing right hand side expression");
    }

    void failMissingOpenParen() const {
        fail("Missing opening parenthesis");
    }

    void failMissingClosingParen() const {
        fail("Missing closing parenthesis");
    }

    void failMissingOpenCurly() const {
        fail("Missing opening curly brace");
    }

    void failMissingClosingCurly() const {
        fail("Missing closing curly brace");
    }

    void failMissingQuot() const {
        fail("Missing quotation mark");
    }

    void failMissingSemi() const {
        fail("Missing semicolon");
    }

    const std::vector<Token> m_tokens;
    size_t m_index = 0;
    TokenType m_lastStmt = {};
    ArenaAllocator m_allocator;
};
