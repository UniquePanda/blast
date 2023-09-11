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
                generator->m_codeSectionAsmOutput << "    mov rax, " << intLitTerm->int_lit.value.value() << "\n";
                generator->pushInternalIntVar("rax");
            }

            void operator()(const StrLitTermNode* strLitTerm) const {
                auto dataLoc = addStringToDataSection(strLitTerm->str_lit.value.value());

                generator->m_codeSectionAsmOutput << "    mov rax, string" << dataLoc << "\n";
                generator->pushConst(
                    "rax",
                    generator->internalConstIdent(dataLoc, "STRING"),
                    false,
                    dataLoc,
                    TokenType::str_lit,
                    strLitTerm->str_lit.value.value().length()
                );
            }

            void operator()(const BoolLitTermNode* boolLitTerm) const {
                generator->m_codeSectionAsmOutput << "    mov rax, " << (boolLitTerm->bool_lit.value.value() == "true" ? 1 : 0) << "\n";
                generator->pushVar(
                    "rax",
                    generator->internalVarIdent("BOOL"),
                    false,
                    TokenType::bool_lit
                );
            }

            void operator()(const IdentTermNode* identTerm) const {
                auto identName = identTerm->ident.value.value();
                bool isConst = generator->m_consts.contains(identName);
                if (!isConst && !generator->m_vars.contains(identName)) {
                    generator->failUndeclaredIdentifer(identName);
                }

                if (isConst) {
                    const auto& cons = generator->m_consts.at(identName);
                    generator->m_codeSectionAsmOutput << "    mov rax, string" << cons.dataLoc << "\n";
                    generator->pushConst("rax", identName, true);
                } else {
                    const auto& var = generator->m_vars.at(identName);
                    std::stringstream offset;
                    offset << "QWORD [rsp + " << (generator->m_stackSize - var.stackLoc - 1) * 8 << "]\n";
                    generator->pushVar(offset.str(), identName, true);
                }
            }

            void operator()(const ParenTermNode* parenTerm) const {
                generator->generateExpr(parenTerm->expr);
            }

            size_t addStringToDataSection(std::string str) const {
                std::vector<std::string> subStrings;

                size_t singleQuotPos = 0;
                while ((singleQuotPos = str.find("'")) != std::string::npos) {
                    subStrings.push_back(str.substr(0, singleQuotPos));
                    str.erase(0, singleQuotPos + 1);
                }
                subStrings.push_back(str);

                generator->m_dataSectionAsmOutput << "    string" << generator->m_dataSize << " db ";
                for (int i = 0; i < subStrings.size(); i++) {
                    generator->m_dataSectionAsmOutput << "'" << subStrings.at(i) << "'";

                    if (i < subStrings.size() - 1) {
                        generator->m_dataSectionAsmOutput << ", \"'\", ";
                    }
                }
                generator->m_dataSectionAsmOutput << ", 0\n";
                generator->m_dataSize++;

                return generator->m_dataSize - 1;
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

                generator->popWithConstNoStrLit("rax");
                generator->popWithConstNoStrLit("rbx");

                generator->m_codeSectionAsmOutput << "    add rax, rbx\n";
                generator->pushInternalIntVar("rax");
            }

            void operator()(const SubBinExprNode* subBinExpr) const {
                generator->generateExpr(subBinExpr->lhs);
                generator->generateExpr(subBinExpr->rhs);

                generator->popWithConstNoStrLit("rbx");
                generator->popWithConstNoStrLit("rax");

                generator->m_codeSectionAsmOutput << "    sub rax, rbx \n";
                generator->pushInternalIntVar("rax");
            }

            void operator()(const MulBinExprNode* mulBinExpr) const {
                generator->generateExpr(mulBinExpr->lhs);
                generator->generateExpr(mulBinExpr->rhs);

                generator->popWithConstNoStrLit("rax");
                generator->popWithConstNoStrLit("rbx");

                generator->m_codeSectionAsmOutput << "    imul rax, rbx\n";
                generator->pushInternalIntVar("rax");
            }

            void operator()(const DivBinExprNode* divBinExpr) const {
                generator->generateExpr(divBinExpr->lhs);
                generator->generateExpr(divBinExpr->rhs);

                generator->popWithConstNoStrLit("rbx"); // Divisor
                generator->popWithConstNoStrLit("rax"); // Divident
                generator->m_codeSectionAsmOutput << "    cqo\n";

                generator->m_codeSectionAsmOutput << "    idiv rbx\n";
                generator->pushInternalIntVar("rax");
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
                    generator->m_codeSectionAsmOutput << "    mov rax, 60\n"; // sys_exit
                    generator->popWithConstNoStrLit("rdi"); // exit code
                    generator->m_codeSectionAsmOutput << "    syscall\n";

                    generator->m_containsCustomExitCall = true;
                } else if (builtInFuncStmt->funcName == "print") {
                    bool isInt = true; // If nothing else can be determien, just assume int.
                    bool isConst = false;

                    if (auto identNameAndIsConst = generator->pop("rdi")) {
                        std::string identName = identNameAndIsConst.value().first;
                        isConst = identNameAndIsConst.value().second;

                        if (isConst) { // If not a const, rdi will hold the value directly
                            Const cons = generator->m_consts.at(identName);
                            if (cons.type == TokenType::int_lit) {
                                generator->m_codeSectionAsmOutput << "    mov rdi, [rdi]\n"; // Just move number into rdi as input for uitoa
                            } else if (cons.type == TokenType::str_lit) {
                                isInt = false;
                                generator->m_codeSectionAsmOutput << "    mov rsi, rdi\n";
                                generator->m_codeSectionAsmOutput << "    mov rdx, " << cons.valueLength << "\n";
                            } else if (cons.type == TokenType::bool_lit) {
                                isInt = false;
                                generator->m_codeSectionAsmOutput << "    call movBoolStr\n"; // rsi points to bool string ("true"/"false"), rdx has string length
                            } else {
                                generator->failUnknownConstType();
                            }
                        } else {
                            Var var = generator->m_vars.at(identName);
                            if (var.type == TokenType::bool_lit) {
                                isInt = false;
                                generator->m_codeSectionAsmOutput << "    call movBoolStr\n"; // rsi points to bool string ("true"/"false"), rdx has string length
                            } else if (var.type != TokenType::int_lit) { // If it's an int, the value is already in rdi
                                generator->failUnknownVarType();
                            }
                        }
                    }

                    if (isInt) {
                        generator->m_codeSectionAsmOutput << "    call uitoa\n"; // rsi points to first char, rdx has string length
                        // Move the stack pointer so the next syscall won't override the string.
                        generator->m_codeSectionAsmOutput << "    sub rsp, rdx\n";
                        generator->m_codeSectionAsmOutput << "    sub rsp, 8\n";
                    }

                    // Perform the stdout write.
                    generator->m_codeSectionAsmOutput << "    mov rdi, 1\n"; // file descriptor 1 (stdout) for sys_write syscall
                    generator->m_codeSectionAsmOutput << "    call sysWrite\n";

                    if (isInt) {
                        // Reset stack pointer.
                        generator->m_codeSectionAsmOutput << "    add rsp, 8\n";
                        generator->m_codeSectionAsmOutput << "    add rsp, rdx\n";
                    }
                } else if (builtInFuncStmt->funcName == "println") {
                    bool isInt = true; // If nothing else can be determien, just assume int.
                    bool isConst = false;

                    if (auto identNameAndIsConst = generator->pop("rdi")) {
                        std::string identName = identNameAndIsConst.value().first;
                        isConst = identNameAndIsConst.value().second;

                        if (isConst) { // If not a const, rdi will hold the value directly
                            Const cons = generator->m_consts.at(identName);
                            if (cons.type == TokenType::int_lit) {
                                generator->m_codeSectionAsmOutput << "    mov rdi, [rdi]\n"; // Just move number into rdi as input for uitoa
                            } else if (cons.type == TokenType::str_lit) {
                                isInt = false;
                                generator->m_codeSectionAsmOutput << "    mov rsi, rdi\n";
                                generator->m_codeSectionAsmOutput << "    mov rdx, " << cons.valueLength << "\n";
                            } else if (cons.type == TokenType::bool_lit) {
                                isInt = false;
                                generator->m_codeSectionAsmOutput << "    call movBoolStr\n"; // rsi points to bool string ("true"/"false"), rdx has string length
                            } else {
                                generator->failUnknownConstType();
                            }
                        } else {
                            Var var = generator->m_vars.at(identName);
                            if (var.type == TokenType::bool_lit) {
                                isInt = false;
                                generator->m_codeSectionAsmOutput << "    call movBoolStr\n"; // rsi points to bool string ("true"/"false"), rdx has string length
                            } else if (var.type != TokenType::int_lit) { // If it's an int, the value is already in rdi
                                generator->failUnknownVarType();
                            }
                        }
                    }

                    if (isInt) {
                        generator->m_codeSectionAsmOutput << "    call uitoa\n"; // rsi points to first char, rdx has string length
                    }

                    // Move rsi before the generated string, add a linebreak and move rsi back. As the string is read
                    // from back to front, this will actually put the linebreak behind the last character.
                    generator->m_codeSectionAsmOutput << "    add rsi, rdx\n";

                    // If we are dealing with a constant, we potentially override important stuff if we add an additional
                    // char. In this case, store the original value and restore it afterwards.
                    // TODO: Probably dangerous? :D
                    if (isConst) {
                        generator->m_codeSectionAsmOutput << "    mov r8b, [rsi]\n";
                    }

                    generator->m_codeSectionAsmOutput << "    mov BYTE [rsi], 0xa\n";
                    generator->m_codeSectionAsmOutput << "    sub rsi, rdx\n";
                    generator->m_codeSectionAsmOutput << "    inc edx\n"; // increase number of chars to read by 1

                    if (isInt) {
                        // Move the stack pointer so the next syscall won't override the string.
                        generator->m_codeSectionAsmOutput << "    sub rsp, rdx\n";
                        generator->m_codeSectionAsmOutput << "    sub rsp, 8\n";
                    }

                    // Perform the stdout write.
                    generator->m_codeSectionAsmOutput << "    mov rdi, 1\n"; // file descriptor 1 (stdout) for sys_write syscall
                    generator->m_codeSectionAsmOutput << "    call sysWrite\n";

                    if (isInt) {
                        // Reset stack pointer.
                        generator->m_codeSectionAsmOutput << "    add rsp, 8\n";
                        generator->m_codeSectionAsmOutput << "    add rsp, rdx\n";
                    }

                    if (isConst) {
                        // Restore value before our string
                        generator->m_codeSectionAsmOutput << "    dec edx\n";
                        generator->m_codeSectionAsmOutput << "    add rsi, rdx\n";
                        generator->m_codeSectionAsmOutput << "    mov [rsi], r8b\n";
                    }
                } else {
                    generator->failUnknownBuiltInFunc(builtInFuncStmt->funcName);
                }
            }

            void operator()(const LetStmtNode* letStmt) const {
                if (
                    generator->m_vars.contains(letStmt->ident.value.value())
                    || generator->m_consts.contains(letStmt->ident.value.value())
                ) {
                    generator->failAlreadyUsedIdentifer(letStmt->ident.value.value());
                }

                // TODO: Quite hacky, but oh well :D
                TokenType tokenType = {};
                size_t valueLength = 0;
                if (std::holds_alternative<TermNode*>(letStmt->expr->var)) {
                    auto term = std::get<TermNode*>(letStmt->expr->var);

                    if (std::holds_alternative<StrLitTermNode*>(term->var)) {
                        tokenType = TokenType::str_lit;
                        valueLength = std::get<StrLitTermNode*>(term->var)->str_lit.value->length();
                    } else if (std::holds_alternative<BoolLitTermNode*>(term->var)) {
                        tokenType = TokenType::bool_lit;
                    } else if (std::holds_alternative<IntLitTermNode*>(term->var)) {
                        tokenType = TokenType::int_lit;
                    }
                } else if (std::holds_alternative<BinExprNode*>(letStmt->expr->var)) {
                    // TODO: This assumes binary expressions are always performed on int_lit
                    tokenType = TokenType::int_lit;
                }

                if (tokenType == TokenType::str_lit) {
                    generator->m_consts.insert({ letStmt->ident.value.value(), Const { .dataLoc = generator->m_dataSize, .type = tokenType, .valueLength = valueLength } });
                } else {
                    generator->m_vars.insert({ letStmt->ident.value.value(), Var { .stackLoc = generator->m_stackSize, .type = tokenType } });
                }

                generator->generateExpr(letStmt->expr);
            }
        };

        StmtVisitor visitor({.generator = this});
        std::visit(visitor, stmt->var);
    }

    [[nodiscard]] std::string generateProg() {
        m_dataSectionAsmOutput << "section .data\n";
        m_dataSectionAsmOutput << "    STRING_TRUE db 'true', 0\n";
        m_dataSectionAsmOutput << "    STRING_FALSE db 'false', 0\n";

        m_codeSectionAsmOutput << "section .text\n";
        m_codeSectionAsmOutput << "global _start\n";
        m_codeSectionAsmOutput << "_start:\n";

        for (const StmtNode* stmt : m_prog.stmts) {
            generateStmt(stmt);
        }

        if (!m_containsCustomExitCall) {
            m_codeSectionAsmOutput << "    mov rax, 60\n";
            m_codeSectionAsmOutput << "    mov rdi, 0\n";
            m_codeSectionAsmOutput << "    syscall\n";
        }

        addUnsignedIntToAsciiAsm();
        addCharacterCountAsm();
        addMovBoolStrAsm();
        addSysWriteAsm();

        m_asmOutput << m_dataSectionAsmOutput.str() << "\n\n" << m_codeSectionAsmOutput.str();

        return m_asmOutput.str();
    }

private:
    void pushVar(
        const std::string& reg,
        const std::string& identName = "",
        const bool& alreadyExists = false,
        const TokenType& type = {}
    ) {
        if (!alreadyExists) {
            m_vars.insert({ identName, Var {.stackLoc = m_stackSize, .type = type}});
        }

        m_varsOnStack.insert({ m_stackSize, identName });

        m_codeSectionAsmOutput << "    push " << reg << "\n";
        m_stackSize++;
    }

    void pushInternalIntVar(const std::string& reg) {
        pushVar(
            reg,
            internalVarIdent("INT"),
            false,
            TokenType::int_lit
        );
    }

    void pushConst(
        const std::string& reg,
        const std::string& identName = "",
        const bool& alreadyExists = false,
        const size_t& dataLoc = 0,
        const TokenType& type = {},
        const size_t& valueLength = 0
    ) {
        if (!alreadyExists) {
            m_consts.insert({ identName, Const { .dataLoc = dataLoc, .type = type, .valueLength = valueLength } });
        }

        m_constsOnStack.insert({ m_stackSize, identName });

        m_codeSectionAsmOutput << "    push " << reg << "\n";
        m_stackSize++;
    }

    std::optional<std::pair<std::string, bool>> pop(const std::string& reg) {
        m_codeSectionAsmOutput << "    pop " << reg << "\n";
        m_stackSize--;

        if (m_constsOnStack.contains(m_stackSize)) {
            std::string constName = m_constsOnStack.at(m_stackSize);
            m_constsOnStack.erase(m_stackSize);
            return std::make_pair(constName, true);
        }

        if (m_varsOnStack.contains(m_stackSize)) {
            std::string varName = m_varsOnStack.at(m_stackSize);
            m_varsOnStack.erase(m_stackSize);
            return std::make_pair(varName, false);
        }

        return {};
    }

    void popWithConstNoStrLit(const std::string& reg) {
        if (auto identNameAndIsConst = pop(reg)) {
            if (identNameAndIsConst.value().second) {
                if (m_consts.at(identNameAndIsConst.value().first).type == TokenType::str_lit) {
                    failOperatorMissmatch();
                }

                m_codeSectionAsmOutput << "    mov " << reg << ", [" << reg << "]\n";
            }
        }
    }

    std::string internalVarIdent(std::string additional = "") {
        m_internalVarsCount++;
        return "<VAR_" + additional + std::to_string(m_internalVarsCount) + ">";
    }

    std::string internalConstIdent(size_t dataLoc, std::string additional = "") {
        return "<CONST_" + additional + std::to_string(dataLoc) + ">";
    }

    /*
    * Input:
    *     edi: integer to convert
    * Output:
    *     rsi (points to first char)
    *     rdx (contains number of chars)
    * Clobbers rsi, rax, rcx, rdx
    */
    void addUnsignedIntToAsciiAsm() {
        // reference: https://stackoverflow.com/a/46301894
        m_codeSectionAsmOutput << "uitoa:\n";

        m_codeSectionAsmOutput << "    mov rax, rdi\n"; // Expects int to convert to be in edi
        m_codeSectionAsmOutput << "    mov ecx, 0xa\n"; // Store 10 as base
        m_codeSectionAsmOutput << "    mov rsi, rsp\n"; // Stack pointer to rsi

        m_codeSectionAsmOutput << ".digitToAsciiLoop:\n";
        m_codeSectionAsmOutput << "    xor edx, edx\n";
        m_codeSectionAsmOutput << "    div rcx\n"; // divide eax by 10, remainder in edx
        m_codeSectionAsmOutput << "    add edx, '0'\n"; // Convert remainder to ascii (adding ascii value of 0)
        m_codeSectionAsmOutput << "    dec rsi\n"; // move stack pointer down
        m_codeSectionAsmOutput << "    mov [rsi], dl\n"; // add lowest byte from edx (ascii value of remainder) to stack
        m_codeSectionAsmOutput << "    test rax, rax\n"; // ZeroFlag set to 1 if eax is 0
        m_codeSectionAsmOutput << "    jnz .digitToAsciiLoop\n"; // process next digit if still number (non-zero) in eax
        m_codeSectionAsmOutput << "    lea edx, [rsp]\n"; // address of rsp to edx
        m_codeSectionAsmOutput << "    sub edx, esi\n"; // subtract address in esi from edx to get length between last and first char
        m_codeSectionAsmOutput << "    ret\n";
        m_codeSectionAsmOutput << "\n";

        // rsi now points to first digit (in ascii) in memory, other digits are stored above in memory
        // rdx now contains the number of chars that were stored in memory
    }

    // TODO: Currently unsued, keep for later :D
    void addCharacterCountAsm() {
        m_codeSectionAsmOutput << "charCount:\n";

        m_codeSectionAsmOutput << ".charCountLoop:\n";
        m_codeSectionAsmOutput << "    mov rsi, rdi\n"; // Expects chars to count to be in rdi
        m_codeSectionAsmOutput << "    xor rdx, rdx\n"; // Set to 0, used as char counter
        m_codeSectionAsmOutput << "    cmp byte [rsi], 0x00\n"; // Expecting null terminating string
        m_codeSectionAsmOutput << "    je .charCountDone\n";
        m_codeSectionAsmOutput << "    inc rdx\n";
        m_codeSectionAsmOutput << "    inc rsi\n";
        m_codeSectionAsmOutput << "    jmp .charCountLoop\n";

        m_codeSectionAsmOutput << ".charCountDone:\n";
        m_codeSectionAsmOutput << "    ret\n";
        m_codeSectionAsmOutput << "\n";
    }

    /*
    * Input:
    *     rdi -> an int that can be converted to bool [0 will be interpreted as false, everything else as true]
    * Output:
    *     rsi -> memory address of begin of correct string for given bool ("false" or "true")
    *     rdx -> length of the string (5 for "false", 4 for "true")
    * Clobbers rsi, rdx
    */
    void addMovBoolStrAsm() {
        m_codeSectionAsmOutput << "movBoolStr:\n";

        m_codeSectionAsmOutput << "    test rdi, rdi\n"; // If 0, it's "false"
        m_codeSectionAsmOutput << "    jnz .mov_bool_str_not_false\n";
        m_codeSectionAsmOutput << "        mov rsi, STRING_FALSE\n";
        m_codeSectionAsmOutput << "        mov rdx, 5\n";
        m_codeSectionAsmOutput << "        ret\n";
        m_codeSectionAsmOutput << "    .mov_bool_str_not_false:\n";
        m_codeSectionAsmOutput << "        mov rsi, STRING_TRUE\n";
        m_codeSectionAsmOutput << "        mov rdx, 4\n";
        m_codeSectionAsmOutput << "        ret\n";
        m_codeSectionAsmOutput << "\n";
    }

    /*
    * Input as defined for sys_write sycall:
    *     rdi -> output file descriptor [1 - stdout, 2 - stderr]
    *     rsi -> pointer to first char
    *     rdx -> length of string
    * Clobbers rax
    */
    void addSysWriteAsm() {
        m_codeSectionAsmOutput << "sysWrite:\n";

        m_codeSectionAsmOutput << "    mov eax, 1\n"; // 1 is sys_write syscall
        m_codeSectionAsmOutput << "    syscall\n";
        m_codeSectionAsmOutput << "    ret\n";
        m_codeSectionAsmOutput << "\n";
    }

    void fail(std::string msg) const {
        std::cerr << msg << std::endl;
        exit(EXIT_FAILURE); 
    }

    void failAlreadyUsedIdentifer(const std::string& identName) const {
        fail("Identifier already used: " + identName);
    }

    void failUndeclaredIdentifer(const std::string& identName) const {
        fail("Undeclared identifier: " + identName);
    }

    void failUnknownBuiltInFunc(const std::string& funcName) const {
        fail("Unknown built in function: " + funcName);
    }

    void failUnknownVarType() const {
        fail("Unknown var type");
    }

    void failUnknownConstType() const {
        fail("Unknown const type");
    }

    void failOperatorMissmatch() const {
        fail("Operator missmatch");
    }

    struct Var {
        size_t stackLoc;
        TokenType type;
    };

    struct Const {
        size_t dataLoc;
        TokenType type;
        size_t valueLength = 0;
    };

    const ProgNode m_prog;
    std::stringstream m_asmOutput;
    std::stringstream m_dataSectionAsmOutput;
    std::stringstream m_codeSectionAsmOutput;
    bool m_containsCustomExitCall = false;
    size_t m_stackSize = 0;
    size_t m_dataSize = 0;
    size_t m_internalVarsCount = 0;
    std::unordered_map<std::string, Var> m_vars {}; // var ident, var
    std::unordered_map<size_t, std::string> m_varsOnStack {}; // stack loc, var ident
    std::unordered_map<std::string, Const> m_consts {}; // const ident, const
    std::unordered_map<size_t, std::string> m_constsOnStack = {}; // stack loc, const ident
};
