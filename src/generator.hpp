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

            void operator()(const ParenTermNode* parenTerm) const {
                generator->generateExpr(parenTerm->expr);
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

            void operator()(const SubBinExprNode* subBinExpr) const {
                generator->generateExpr(subBinExpr->lhs);
                generator->generateExpr(subBinExpr->rhs);

                generator->pop("rax");
                generator->pop("rbx");

                generator->m_asmOutput << "    sub rbx, rax \n";
                generator->push("rbx");
            }

            void operator()(const MulBinExprNode* mulBinExpr) const {
                generator->generateExpr(mulBinExpr->lhs);
                generator->generateExpr(mulBinExpr->rhs);

                generator->pop("rax");
                generator->pop("rbx");

                generator->m_asmOutput << "    imul rax, rbx\n";
                generator->push("rax");
            }

            void operator()(const DivBinExprNode* divBinExpr) const {
                generator->generateExpr(divBinExpr->lhs);
                generator->generateExpr(divBinExpr->rhs);

                generator->pop("rbx"); // Divisor
                generator->pop("rax"); // Divident
                generator->m_asmOutput << "    cqo\n";

                generator->m_asmOutput << "    idiv rbx\n";
                generator->push("rax");
                // TODO: Remainder is in rdx. Can't be used right now anyways as we only print exit code.
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

            void operator()(const BuiltInFuncStmtNode* builtInFuncStmt) const {
                generator->generateExpr(builtInFuncStmt->expr);

                if (builtInFuncStmt->funcName == "exit") {
                    generator->m_asmOutput << "    mov rax, 60\n"; // sys_exit
                    generator->pop("rdi"); // exit code
                    generator->m_asmOutput << "    syscall\n";

                    generator->m_containsCustomExitCall = true;
                } else {
                    generator->failUnknownBuiltInFunc(builtInFuncStmt->funcName);
                }
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

        if (!m_containsCustomExitCall) {
            m_asmOutput << "    mov rax, 60\n";
            m_asmOutput << "    mov rdi, 0\n";
            m_asmOutput << "    syscall\n";
        }

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

    void failUnknownBuiltInFunc(std::string funcName) const {
        fail("Unknown built in function: " + funcName);
    }

    struct Var {
        size_t stackLoc;
    };

    const ProgNode m_prog;
    std::stringstream m_asmOutput;
    bool m_containsCustomExitCall = false;
    size_t m_stackSize = 0;
    std::unordered_map<std::string, Var> m_vars {};
};
