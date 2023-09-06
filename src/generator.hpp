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
                } else if (builtInFuncStmt->funcName == "print") {
                    generator->pop("rdi"); // Input for uitoa
                    generator->m_asmOutput << "    call uitoa\n"; // rsi points to first char, rdx has string length
                    generator->m_asmOutput << "    mov rdi, 1\n"; // file descriptor 1 (stdout) for sys_wrtie syscall
                    generator->m_asmOutput << "    call sysWrite\n";
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
        addUnsignedIntToAsciiAsm();
        addCharacterCountAsm();
        addSysWriteAsm();

        m_asmOutput << "global _start\n";
        m_asmOutput << "_start:\n";

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

    /*
    * Input in edi
    * Output:
    *     rsi (points to first char) -> overwritten by stack pushes!
    *     rdx (contains number of chars)
    * Clobbers rsi, rax, rcx, rdx
    * Pushes to stack: 1
    */
    void addUnsignedIntToAsciiAsm() {
        // reference: https://stackoverflow.com/a/46301894
        m_asmOutput << "uitoa:\n";

        m_asmOutput << "    mov eax, edi\n"; // Expects int to convert to be in edi
        m_asmOutput << "    mov ecx, 0xa\n"; // Store 10 as base
        m_asmOutput << "    mov rsi, rsp\n"; // Stack pointer to rsi

        m_asmOutput << ".digitToAsciiLoop:\n";
        m_asmOutput << "    xor edx, edx\n";
        m_asmOutput << "    div ecx\n"; // divide eax by 10, remainder in edx
        m_asmOutput << "    add edx, '0'\n"; // Convert remainder to ascii (adding ascii value of 0)
        m_asmOutput << "    dec rsi\n"; // move stack pointer down
        m_asmOutput << "    mov [rsi], dl\n"; // add lowest byte from edx (ascii value of remainder) to stack
        m_asmOutput << "    test eax, eax\n"; // ZeroFlag set to 1 if eax is 0
        m_asmOutput << "    jnz .digitToAsciiLoop\n"; // process next digit if still number (non-zero) in eax
        m_asmOutput << "    lea edx, [rsp]\n"; // address of rsp + 1 (address of the \n char) to ecx
        m_asmOutput << "    sub edx, esi\n"; // subtract address in exi from ecx to get length between last and first char
        m_asmOutput << "    ret\n";
        m_asmOutput << "\n";

        // rsi now points to first digit (in ascii) in memory, other digits are stored above in memory
        // rdx now contains the number of chars that were stored in memory
    }

    // TODO: Currently unsued, keep for later :D
    void addCharacterCountAsm() {
        m_asmOutput << "charCount:\n";

        m_asmOutput << ".charCountLoop:\n";
        m_asmOutput << "    mov rsi, rdi\n"; // Expects chars to count to be in rdi
        m_asmOutput << "    xor rdx, rdx\n"; // Set to 0, used as char counter
        m_asmOutput << "    cmp byte [rsi], 0x00\n"; // Expecting null terminating string
        m_asmOutput << "    je .charCountDone\n";
        m_asmOutput << "    inc rdx\n";
        m_asmOutput << "    inc rsi\n";
        m_asmOutput << "    jmp .charCountLoop\n";

        m_asmOutput << ".charCountDone:\n";
        m_asmOutput << "    ret\n";
        m_asmOutput << "\n";
    }

    /*
    * Input as defined for sys_write sycall:
    *     rdi -> output file descriptor [1 - stdout, 2 - stderr]
    *     rsi -> pointer to first char
    *     rdx -> length of string
    * Clobbers rax
    */
    void addSysWriteAsm() {
        m_asmOutput << "sysWrite:\n";

        m_asmOutput << "    mov eax, 1\n"; // 1 is sys_write syscall
        m_asmOutput << "    syscall\n";
        m_asmOutput << "    ret\n";
        m_asmOutput << "\n";
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
