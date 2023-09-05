#pragma once

#include <iostream>
#include <optional>
#include <vector>

enum class TokenType {
    let, ident, int_lit, built_in_func,
    eq, open_paren, close_paren, semi,
    plus, minus, star, slash,
};

struct Token {
    static const size_t MAX_PRECEDENCE = 2;

    TokenType type;
    std::optional<std::string> value {};
    size_t precedence = MAX_PRECEDENCE;
};

const std::vector<std::string> BUILT_IN_FUNC_NAMES {"exit"};

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
                std::cerr << "Unknown character <" << peek().value() << ">" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        m_index = 0;
        return tokens;
    }

private:
    [[nodiscard]] std::optional<char> peek(int offset = 0) const {
        if (m_index + offset >= m_source.length()) {
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

    const std::string m_source;
    size_t m_index = 0;
};
