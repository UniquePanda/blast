#pragma once

#include <iostream>
#include <optional>
#include <vector>

enum class TokenType {
    exit, let, ident, int_lit,
    eq, open_paren, close_paren, semi,
    plus, star,
};

struct Token {
    static const size_t MAX_PRECEDENCE = 2;

    TokenType type;
    std::optional<std::string> value {};
    size_t precedence = MAX_PRECEDENCE;
};

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

                if (buf == "exit") {
                    tokens.push_back({ .type = TokenType::exit});
                    buf.clear();
                    continue;
                } else if (buf == "let") {
                    tokens.push_back({ .type = TokenType::let });
                    buf.clear();
                    continue;
                } else {
                    tokens.push_back({ .type = TokenType::ident, .value = buf });
                    buf.clear();
                    continue;
                }
            } else if (std::isdigit(peek().value())) {
                buf.push_back(consume());

                while (peek().has_value() && std::isdigit(peek().value())) {
                    buf.push_back(consume());
                }

                tokens.push_back({.type = TokenType::int_lit, .value = buf});
                buf.clear();
                continue;
            } else if (peek().value() == '=') {
                tokens.push_back({.type = TokenType::eq});
                consume();
                continue;
            } else if (peek().value() == '(') {
                consume();
                tokens.push_back({ .type = TokenType::open_paren });
                continue;
            }else if (peek().value() == ')') {
                consume();
                tokens.push_back({ .type = TokenType::close_paren });
                continue;
            } else if (peek().value() == ';') {
                tokens.push_back({ .type = TokenType::semi });
                consume();
                continue;
            } else if (peek().value() == '+') {
                tokens.push_back({ .type = TokenType::plus, .precedence = 0 });
                consume();
                continue;
            } else if (peek().value() == '*') {
                tokens.push_back({ .type = TokenType::star, .precedence = 1 });
                consume();
                continue;
            } else if (std::isspace(peek().value())) {
                consume();
                continue;
            }

            std::cerr << "Unknown character <" << peek().value() << ">" << std::endl;
            exit(EXIT_FAILURE);
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

    const std::string m_source;
    size_t m_index = 0;
};
