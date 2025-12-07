#!/usr/bin/env python3
"""
LaTeX Equation Validator for Robotics Content
Validates mathematical expressions and equations in markdown files
"""

import re
import sys
import os
from pathlib import Path
from typing import List, Tuple, Dict


class MathValidator:
    def __init__(self):
        self.inline_math_pattern = r"\$([^$]+)\$"
        self.display_math_pattern = r"\$\$([^$]+)\$\$"
        self.equation_errors = []
        self.warnings = []

    def validate_file(self, file_path: Path) -> Dict:
        """Validate mathematical expressions in a markdown file"""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            return {"error": f"Could not read file: {e}"}

        # Find all mathematical expressions
        inline_expressions = re.findall(self.inline_math_pattern, content)
        display_expressions = re.findall(self.display_math_pattern, content)

        errors = []
        warnings = []

        # Validate each expression
        for expr in inline_expressions:
            error = self._validate_expression(expr, "inline")
            if error:
                errors.append(f"Inline math: {error}")

        for expr in display_expressions:
            error = self._validate_expression(expr, "display")
            if error:
                errors.append(f"Display math: {error}")

        # Check for common issues
        self._check_common_issues(content, warnings)

        return {
            "file": str(file_path),
            "inline_count": len(inline_expressions),
            "display_count": len(display_expressions),
            "errors": errors,
            "warnings": warnings,
            "valid": len(errors) == 0,
        }

    def _validate_expression(self, expr: str, expr_type: str) -> str:
        """Validate individual mathematical expression"""
        expr = expr.strip()

        # Check for unbalanced braces
        if expr.count("{") != expr.count("}"):
            return "Unbalanced braces"

        # Check for unbalanced parentheses
        if expr.count("(") != expr.count(")"):
            return "Unbalanced parentheses"

        # Check for unbalanced brackets
        if expr.count("[") != expr.count("]"):
            return "Unbalanced brackets"

        # Check for common LaTeX syntax errors
        if "\\frac" in expr:
            if not re.search(r"\\frac\{[^}]+\}\{[^}]+\}", expr):
                return "Invalid fraction syntax"

        if "\\sqrt" in expr and "[" in expr:
            if not re.search(r"\\sqrt\[[^\]]+\]", expr):
                return "Invalid square root syntax"

        # Check for undefined commands (basic check)
        undefined_commands = []
        for match in re.finditer(r"\\([a-zA-Z]+)", expr):
            cmd = match.group(1)
            if cmd not in self._get_allowed_commands():
                undefined_commands.append(cmd)

        if undefined_commands:
            return f"Potentially undefined commands: {', '.join(undefined_commands)}"

        return None

    def _get_allowed_commands(self) -> set:
        """Get set of commonly allowed LaTeX commands"""
        return {
            "frac",
            "sqrt",
            "sum",
            "prod",
            "int",
            "partial",
            "nabla",
            "infty",
            "alpha",
            "beta",
            "gamma",
            "delta",
            "epsilon",
            "zeta",
            "eta",
            "theta",
            "iota",
            "kappa",
            "lambda",
            "mu",
            "nu",
            "xi",
            "pi",
            "rho",
            "sigma",
            "tau",
            "upsilon",
            "phi",
            "chi",
            "psi",
            "omega",
            "Gamma",
            "Delta",
            "Theta",
            "Lambda",
            "Xi",
            "Pi",
            "Sigma",
            "Phi",
            "Psi",
            "Omega",
            "sin",
            "cos",
            "tan",
            "log",
            "ln",
            "exp",
            "max",
            "min",
            "lim",
            "sup",
            "inf",
            "det",
            "tr",
            "mathbf",
            "mathit",
            "mathrm",
            "mathcal",
            "mathbb",
            "mathfrak",
            "begin",
            "end",
            "left",
            "right",
            "over",
            "choose",
            "binom",
            "matrix",
            "pmatrix",
            "bmatrix",
            "vmatrix",
            "Vmatrix",
            "text",
            "mathrm",
            "vec",
            "dot",
            "ddot",
            "bar",
            "hat",
            "tilde",
        }

    def _check_common_issues(self, content: str, warnings: List[str]):
        """Check for common mathematical content issues"""
        # Check for missing equation numbers in display math
        display_eqs = re.findall(r"\$\$([^$]+)\$\$", content)
        numbered_eqs = re.findall(r"\$\$([^$]+)\$\$\s*\\label\{[^}]+\}", content)

        if len(display_eqs) > len(numbered_eqs) + 2:  # Allow some unnumbered
            warnings.append("Consider adding equation numbers to display equations")

        # Check for mathematical notation without LaTeX
        plain_math_patterns = [
            r"\btheta\b",
            r"\balpha\b",
            r"\bbeta\b",
            r"\bgamma\b",
            r"\bdelta\b",
            r"\bpi\b",
            r"\bsigma\b",
            r"\bomega\b",
        ]

        for pattern in plain_math_patterns:
            if re.search(pattern, content) and not re.search(
                r"\$" + pattern[1:] + r"\$", content
            ):
                warnings.append(f"Potential un-LaTeXed math notation: {pattern}")

    def validate_directory(self, directory: Path) -> Dict:
        """Validate all markdown files in a directory"""
        results = []

        for md_file in directory.rglob("*.md"):
            if md_file.is_file():
                result = self.validate_file(md_file)
                results.append(result)

        total_errors = sum(len(r.get("errors", [])) for r in results)
        total_warnings = sum(len(r.get("warnings", [])) for r in results)
        valid_files = sum(1 for r in results if r.get("valid", False))

        return {
            "directory": str(directory),
            "total_files": len(results),
            "valid_files": valid_files,
            "total_errors": total_errors,
            "total_warnings": total_warnings,
            "results": results,
        }


def main():
    if len(sys.argv) != 2:
        print("Usage: python math-validator.py <directory>")
        sys.exit(1)

    directory = Path(sys.argv[1])
    if not directory.exists():
        print(f"Error: Directory {directory} does not exist")
        sys.exit(1)

    validator = MathValidator()
    result = validator.validate_directory(directory)

    print(f"\nMathematical Validation Results for {directory}")
    print("=" * 50)
    print(f"Total files: {result['total_files']}")
    print(f"Valid files: {result['valid_files']}")
    print(f"Total errors: {result['total_errors']}")
    print(f"Total warnings: {result['total_warnings']}")

    if result["total_errors"] > 0:
        print("\nERRORS:")
        for file_result in result["results"]:
            if file_result.get("errors"):
                print(f"\n{file_result['file']}:")
                for error in file_result["errors"]:
                    print(f"  - {error}")

    if result["total_warnings"] > 0:
        print("\nWARNINGS:")
        for file_result in result["results"]:
            if file_result.get("warnings"):
                print(f"\n{file_result['file']}:")
                for warning in file_result["warnings"]:
                    print(f"  - {warning}")

    # Return appropriate exit code
    sys.exit(0 if result["total_errors"] == 0 else 1)


if __name__ == "__main__":
    main()
