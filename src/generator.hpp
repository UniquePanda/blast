#pragma once

#include <cassert>
#include <sstream>
#include <unordered_map>

#include "parser.hpp"

class Generator {
public:
    Generator(ProgNode prog) : m_prog(std::move(prog)) {}

    void generateTerm(const TermNode* term) {
        struct TermVisitor {
            Generator* generator;

            void operator()(const IntLitTermNode* intLitTerm) const {
                generator->m_asmOutput << "    mov rax, " << intLitTerm->int_lit.value.value() << "\n";
                generator->push("rax");
            }

            void operator()(const IdentTermNode* identTerm) const {
                if (!generator->m_vars.contains(identTerm->ident.value.value())) {
                    generator->failUndeclaredIdentifer(identTerm->ident.value.value());
                }

                const auto& var = generator->m_vars.at(identTerm->ident.value.value());
                std::stringstream offset;
                offset << "QWORD [rsp + " << (generator->m_stackSize - var.stackLoc - 1) * 8 << "]\n";
                generator->push(offset.str());
            }
        };

        TermVisitor visitor({ .generator = this });
        std::visit(visitor, term->var);
    }

    void generateExpr(const ExprNode* expr) {
        struct BinExprVisitor {
            Generator* generator;

            void operator()(const SumBinExprNode* sumBinExpr) const {
                generator->generateExpr(sumBinExpr->lhs);
                generator->generateExpr(sumBinExpr->rhs);

                generator->pop("rax");
                generator->pop("rbx");

                generator->m_asmOutput << "    add rax, rbx\n";
                generator->push("rax");
            }

            void operator()(const MulBinExprNode* mulBinExpr) const {
                generator->fail("Multiplication not yet implemented");
            }
        };

        struct ExprVisitor {
            Generator* generator;

            void operator()(const TermNode* term) const {
                generator->generateTerm(term);
            }

            void operator()(const BinExprNode* binExpr) const {
                BinExprVisitor visitor({.generator = generator});
                std::visit(visitor, binExpr->var);
            }
        };

        ExprVisitor visitor({.generator = this});
        std::visit(visitor, expr->var);
    }

    void generateStmt(const StmtNode* stmt) {
        struct StmtVisitor {
            Generator* generator;

            void operator()(const ExitStmtNode* exitStmt) const {
                generator->generateExpr(exitStmt->expr);

                generator->m_asmOutput << "    mov rax, 60\n";
                generator->pop("rdi");
                generator->m_asmOutput << "    syscall\n";
            }

            void operator()(const LetStmtNode* letStmt) const {
                if (generator->m_vars.contains(letStmt->ident.value.value())) {
                    generator->failAlreadyUsedIdentifer(letStmt->ident.value.value());
                }

                generator->m_vars.insert({ letStmt->ident.value.value(), Var { .stackLoc = generator->m_stackSize } });
                generator->generateExpr(letStmt->expr);
            }
        };

        StmtVisitor visitor({.generator = this});
        std::visit(visitor, stmt->var);
    }

    [[nodiscard]] std::string generateProg() {
        m_asmOutput << "global _start\n_start:\n";

        for (const StmtNode* stmt : m_prog.stmts) {
            generateStmt(stmt);
        }

        m_asmOutput << "    mov rax, 60\n";
        m_asmOutput << "    mov rdi, 0\n";
        m_asmOutput << "    syscall\n";

        return m_asmOutput.str();
    }

private:
    void push(const std::string& reg) {
        m_asmOutput << "    push " << reg << "\n";
        m_stackSize++;
    }

    void pop(const std::string& reg) {
        m_asmOutput << "    pop " << reg << "\n";
        m_stackSize--;
    }

    void fail(std::string msg) const {
        std::cerr << msg << std::endl;
        exit(EXIT_FAILURE); 
    }

    void failAlreadyUsedIdentifer(std::string identName) const {
        fail("Identifier already used: " + identName);
    }

    void failUndeclaredIdentifer(std::string identName) const {
        fail("Undeclared identifier: " + identName);
    }

    struct Var {
        size_t stackLoc;
    };

    const ProgNode m_prog;
    std::stringstream m_asmOutput;
    size_t m_stackSize = 0;
    std::unordered_map<std::string, Var> m_vars {};
};
