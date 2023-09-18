#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "generator.hpp"
#include "parser.hpp"
#include "tokenizer.hpp"

int main(int argc, char const* argv[]) {
    if (argc != 2) {
        std::cerr << "Incorrect usage. Correct usage is:" << std::endl;
        std::cerr << "bla <input.bla>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string contents;

    {
        std::fstream input(argv[1], std::ios::in);
        std::stringstream contents_stream;
        contents_stream << input.rdbuf();
        input.close();

        contents = contents_stream.str();
    }

    Tokenizer tokenizer(std::move(contents));
    std::vector<Token> tokens = tokenizer.tokenize();
    std::cout << "      Tokenization successful" << std::endl;

    Parser parser(std::move(tokens));
    std::optional<ProgNode> tree = parser.parseProg();
    std::cout << "      Parsing successful" << std::endl;

    if (!tree.has_value()) {
        std::cerr << "No exit statement found" << std::endl;
        return EXIT_FAILURE;
    }

    Generator generator(tree.value());

    {
        std::fstream file("../blaOut/out.asm", std::ios::out);
        file << generator.generateProg();
        std::cout << "      Generation successful" << std::endl;
        file.close();

        system("nasm -felf64 ../blaOut/out.asm");
        system("ld -o ../blaOut/out ../blaOut/out.o");
    }
    std::cout << "      Compiling successful\n|==============================|\n" << std::endl;

    return EXIT_SUCCESS;
}
