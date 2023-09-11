$$

\begin{align}
    [\text{Prog}] &\to [\text{Stmt}]^* \\
    [\text{Stmt}] &\to
        \begin{cases}
            \text{let}\space\text{ident} = \text{[Expr]}; \\
            \text{[BuiltInFunc]}(\text{[Expr]}); \\
        \end{cases}
    \\
    [\text{Expr}] &\to
        \begin{cases}
            \text{SumBinExpr} \\
            \text{SubBinExpr} \\
            \text{MulBinExpr} \\
            \text{Term} \\
        \end{cases}
    \\
    [\text{Term}] &\to
        \begin{cases}
            \text{ident} \\
            \text{int\_lit} \\
            "\text{str\_lit}" \\
            \text{bool\_lit} \\
            ([\text{Expr}]) \\
        \end{cases}
    \\
    [\text{SumBinExpr}] &\to
        \begin{cases}
            [\text{Expr}] + [\text{Expr}] \\
            [\text{Expr}] + [\text{MulBinExpr}] \\
        \end{cases}
    \\
    [\text{SubBinExpr}] &\to
        \begin{cases}
            [\text{Expr}] - [\text{Expr}] \\
            [\text{Expr}] - [\text{MulBinExpr}] \\
        \end{cases}
    \\
    [\text{MulBinExpr}] &\to [\text{Expr}] * [\text{Expr}] \\
    [\text{BuiltInFunc}] &\to
        \begin{cases}
            \text{exit} & \text{Exit program with given exit code} \\
            \text{print} & \text{Print given input to stdout} \\
            \text{println} & \text{Print given input with added line break to stdout} \\
        \end{cases}
    \\
\end{align}

$$