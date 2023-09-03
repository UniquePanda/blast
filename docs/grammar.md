$$

\begin{align}
    [\text{Prog}] &\to [\text{Stmt}]^* \\
    [\text{Stmt}] &\to
        \begin{cases}
            \text{let}\space\text{ident} = \text{[Expr]}; \\
            \text{exit}(\text{[Expr]}); \\
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
\end{align}

$$