#pragma once

#include <iostream>
#include <optional>
#include <vector>

enum class TokenType {
    unknown,
    let, ident, int_lit, str_lit, bool_lit, built_in_func,
    eq, open_paren, close_paren, semi, quot,
    plus, minus, star, slash
};

struct Token {
    static const size_t MAX_PRECEDENCE = 2;

    TokenType type;
    std::optional<std::string> value {};
    size_t precedence = MAX_PRECEDENCE;
};

const std::vector<std::string> BUILT_IN_FUNC_NAMES {"print", "println", "exit"};

class Tokenizer {
public:
    Tokenizer(const std::string source) : m_source(std::move(source)) {}

    std::vector<Token> tokenize() {
        std::vector<Token> tokens;
        std::string buf;

        while (peek().has_value()) {
            if (std::isalpha(peek().value())) {
                buf.push_back(consume());

                while (peek().has_value() && std::isalnum(peek().value())) {
                    buf.push_back(consume());
                }

                if (buf == "let") {
                    tokens.push_back({ .type = TokenType::let });
                } else if (buf == "true" || buf == "false") {
                    tokens.push_back({ .type = TokenType::bool_lit, .value = buf });
                } else if (auto builtInFuncToken = evaluateBuiltInFunc(buf)) {
                    tokens.push_back(builtInFuncToken.value());
                } else {
                    tokens.push_back({ .type = TokenType::ident, .value = buf });
                }

                buf.clear();
            } else if (std::isdigit(peek().value())) {
                buf.push_back(consume());

                while (peek().has_value() && std::isdigit(peek().value())) {
                    buf.push_back(consume());
                }

                tokens.push_back({.type = TokenType::int_lit, .value = buf});
                buf.clear();
            } else if (peek().value() == '"') {
                tokens.push_back({.type = TokenType::quot});
                consume();
                // Consume until next quotation mark, because it's a string.
                while (peek().has_value()) {
                    if (peek().value() == '\\') {
                        int slashCount = 0;
                        while (peek().has_value() && peek().value() == '\\') {
                            slashCount++;
                            if (slashCount % 2 == 0) {
                                buf.push_back(consume());
                            } else {
                                consume();
                            }
                        }

                        if (slashCount % 2 != 0) {
                            if (peek().value() == '"') {
                                buf.push_back(consume());
                                continue;
                            } else {
                                failUnxpectedEscape();
                            }
                        }
                    }

                    if (peek().value() == '"') {
                        break;
                    }

                    buf.push_back(consume());
                }

                tokens.push_back({.type = TokenType::str_lit, .value = buf});
                buf.clear();

                // Consume ending quotation mark.
                if (peek().has_value()) {
                    tokens.push_back({.type = TokenType::quot});
                    consume();
                }
            } else if (peek().value() == '=') {
                tokens.push_back({.type = TokenType::eq});
                consume();
            } else if (peek().value() == '(') {
                consume();
                tokens.push_back({ .type = TokenType::open_paren });
            }else if (peek().value() == ')') {
                consume();
                tokens.push_back({ .type = TokenType::close_paren });
            } else if (peek().value() == ';') {
                tokens.push_back({ .type = TokenType::semi });
                consume();
            } else if (peek().value() == '+') {
                tokens.push_back({ .type = TokenType::plus, .precedence = 0 });
                consume();
            } else if (peek().value() == '-') {
                tokens.push_back({ .type = TokenType::minus, .precedence = 0 });
                consume();
            } else if (peek().value() == '*') {
                tokens.push_back({ .type = TokenType::star, .precedence = 1 });
                consume();
            } else if (peek().value() == '/') {
                if (peek(1).value() == '/') {
                    // Discard until new line, because it's a comment
                    while (peek().has_value() && peek().value() != '\n') {
                        consume();
                    }
                } else if (peek(1).value() == '*') {
                    // Discard until */ because it's a multiline comment
                    while (peek().has_value() && peek(1).has_value()) {
                        if (peek().value() == '*' && peek(1).value() == '/') {
                            consume();
                            consume();
                            break;
                        }

                        consume();
                    }
                } else {
                    tokens.push_back({ .type = TokenType::slash, .precedence = 1 });
                    consume();
                }
            } else if (std::isspace(peek().value())) {
                consume();
            } else {
                failUnknownChar(peek().value());
            }
        }

        m_index = 0;
        return tokens;
    }

private:
    [[nodiscard]] std::optional<char> peek(int offset = 0) const {
        if (m_index + offset >= 0 && m_index + offset >= m_source.length()) {
            return {};
        } else {
            return m_source.at(m_index + offset);
        }
    }

    char consume() {
        return m_source.at(m_index++);
    }

    std::optional<Token> evaluateBuiltInFunc(std::string funcName) {
        if (std::find(BUILT_IN_FUNC_NAMES.begin(), BUILT_IN_FUNC_NAMES.end(), funcName) == BUILT_IN_FUNC_NAMES.end()) {
            return {};
        }

        Token token;
        token.type = TokenType::built_in_func;
        token.value = funcName;
        return token;
    }

    void fail(std::string msg) const {
        std::cerr << msg << std::endl;
        exit(EXIT_FAILURE); 
    }

    void failUnxpectedEscape() const {
        fail("Unexpected escape sequence");
    }

    void failUnknownChar(char unknownChar) const {
        fail("Unknown character: " + std::string(1, unknownChar));
    }

    const std::string m_source;
    size_t m_index = 0;
};
