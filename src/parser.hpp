#pragma once

#include <variant>
#include <vector>
#include <optional>

#include "arenaAllocator.hpp"
#include "tokenizer.hpp"



struct IntLitTermNode {
    Token int_lit;
};

struct IdentTermNode {
    Token ident;
};

struct ExprNode;

struct SumBinExprNode {
    ExprNode* lhs;
    ExprNode* rhs;
};

struct MulBinExprNode {
    ExprNode* lhs;
    ExprNode* rhs;
};

struct BinExprNode {
    std::variant<SumBinExprNode*, MulBinExprNode*> var;
};

struct TermNode {
    std::variant<IdentTermNode*, IntLitTermNode*> var;
};

struct ExprNode {
    std::variant<TermNode*, BinExprNode*> var;
};

struct ExitStmtNode {
    ExprNode* expr;
};

struct LetStmtNode {
    Token ident;
    ExprNode* expr;
};

struct StmtNode {
    std::variant<ExitStmtNode*, LetStmtNode*> var;
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
         if (!peek().has_value()) {
            return {};
        }

        if (peek().value().type == TokenType::int_lit) {
            auto intLitTerm = m_allocator.alloc<IntLitTermNode>();
            intLitTerm->int_lit = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = intLitTerm;
            return term;
        } else if (peek().value().type == TokenType::ident) {
            auto identTerm = m_allocator.alloc<IdentTermNode>();
            identTerm->ident = consume();
            auto term = m_allocator.alloc<TermNode>();
            term->var = identTerm;
            return term;
        } else {
            return {};
        }
    }

    std::optional<ExprNode*> parseExpr(size_t precedence = 0) {
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

            if (peek().value().type == TokenType::plus) {
                auto binExpr = m_allocator.alloc<BinExprNode>();
                auto sumBinExpr = m_allocator.alloc<SumBinExprNode>();
                sumBinExpr->lhs = lhsExpr.value();
                consume();

                if (auto rhs = parseExpr(precedence)) {
                    sumBinExpr->rhs = rhs.value();
                    binExpr->var = sumBinExpr;
                    auto expr = m_allocator.alloc<ExprNode>();
                    expr->var = binExpr;
                    return expr;
                } else {
                    failBinaryRHS();
                }
            } else if(peek().value().type == TokenType::star) {
                auto binExpr = m_allocator.alloc<BinExprNode>();
                auto mulBinExpr = m_allocator.alloc<MulBinExprNode>();
                mulBinExpr->lhs = lhsExpr.value();
                consume();

                if (auto rhs = parseExpr(precedence)) {
                    mulBinExpr->rhs = rhs.value();
                    binExpr->var = mulBinExpr;
                    auto expr = m_allocator.alloc<ExprNode>();
                    expr->var = binExpr;
                    return expr;
                } else {
                    failBinaryRHS();
                }
            } else {
                failUnsupportedBinaryOperator();
            }
        } else {
            return {};
        }

        return {};
    }

    std::optional<StmtNode*> parseStmt() {
        if (!peek().has_value()) {
            return {};
        }

        if (peek().value().type == TokenType::exit) {
            if (!peek(1).has_value()) {
                failUnexpectedEOF();
            }

            if (peek(1).value().type != TokenType::open_paren) {
                failMissingOpenParen();
            }

            // exit and opening parenthesis
            consume();
            consume();

            auto exitStmt = m_allocator.alloc<ExitStmtNode>();
            if (auto exprNode = parseExpr()) {
                exitStmt->expr = exprNode.value();
            } else {
                failInvalidExpr();
            }
            if (peek().has_value() && peek().value().type == TokenType::close_paren) {
                consume();
            } else {
                failMissingClosingParen();
            }

            consumeSemi();

            if (peek().has_value()) {
                std::cerr << "Found code after exit()" << std::endl;
                exit(EXIT_FAILURE);
            }

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = exitStmt;
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

            auto stmt = m_allocator.alloc<StmtNode>();
            stmt->var = letStmt;
            return stmt;
        } else {
            return {};
        }
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

    void failInvalidExpr() const {
        fail("Invalid expression");
    }

    void failMissingIdent() const {
        fail("Missing identifier");
    }

    void failMissingOperator() const {
        fail("Missing operator");
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

    void failMissingSemi() const {
        fail("Missing semicolon");
    }

    const std::vector<Token> m_tokens;
    size_t m_index = 0;
    ArenaAllocator m_allocator;
};
