$$

\begin{align}
    [\text{Prog}] &\to [\text{Stmt}]^* \\
    [\text{Stmt}] &\to
        \begin{cases}
            \text{let}\space\text{ident} = \text{[Expr]}; \\
            \text{[BuiltInFunc]} \ \text{'('} \ \text{[Expr]} \ \text{')'}; \\
            \text{if} \ \text{'('} \ \text{[Expr]} \ \text{')'} \ [\text{Scope}] \\
            \quad (\text{elseif} \ \text{'('} \ \text{[Expr]} \ \text{')'} \ [\text{Scope}])^* \\
            \quad (\text{else} \ [\text{Scope}])? \\
            \text{[Scope]} \\
        \end{cases}
    \\
    [\text{Scope}] &\to \{[\text{Stmt}]^*\}\ \\
    [\text{Expr}] &\to
        \begin{cases}
            \text{SumBinExpr} \\
            \text{SubBinExpr} \\
            \text{MulBinExpr} \\
            \text{DivBinExpr} \\
            \text{Term} \\
        \end{cases}
    \\
    [\text{Term}] &\to
        \begin{cases}
            \text{ident} \\
            \text{int\_lit} \\
            \text{dbl\_lit} \\
            \text{str\_lit} \\
            \text{bool\_lit} \\
            \text{'('} \ [\text{Expr}] \ \text{')'} \\
        \end{cases}
    \\
    [\text{SumBinExpr}] &\to
        \begin{cases}
            [\text{Expr}] + [\text{Expr}] \\
            [\text{Expr}] + [\text{MulBinExpr}] \\
            [\text{Expr}] + [\text{DivBinExpr}] \\
        \end{cases}
    \\
    [\text{SubBinExpr}] &\to
        \begin{cases}
            [\text{Expr}] - [\text{Expr}] \\
            [\text{Expr}] - [\text{MulBinExpr}] \\
            [\text{Expr}] - [\text{DivBinExpr}] \\
        \end{cases}
    \\
    [\text{MulBinExpr}] &\to [\text{Expr}] * [\text{Expr}] \\
    [\text{DivBinExpr}] &\to [\text{Expr}] / [\text{Expr}] \\
    [\text{BuiltInFunc}] &\to
        \begin{cases}
            \text{exit} & \text{Exit program with given exit code} \\
            \text{print} & \text{Print given input to stdout} \\
            \text{println} & \text{Print given input with added line break to stdout} \\
        \end{cases}
    \\
\end{align}

$$