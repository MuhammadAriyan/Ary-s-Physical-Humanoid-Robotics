#!/usr/bin/env python3
"""
Code Snippet Tester for Robotics Content
Tests Python, MATLAB, and ROS2 code examples for syntax and execution
"""

import subprocess
import sys
import os
import tempfile
import json
from pathlib import Path
from typing import Dict, List, Any


class CodeTester:
    def __init__(self):
        self.supported_languages = {
            "python": {
                "extension": ".py",
                "command": ["python3"],
                "syntax_check": ["python3", "-m", "py_compile"],
            },
            "matlab": {
                "extension": ".m",
                "command": ["matlab", "-batch", "-nodisplay"],
                "syntax_check": ["matlab", "-batch", "-nodisplay", "-r"],
            },
            "ros2": {
                "extension": ".py",
                "command": ["python3"],
                "syntax_check": ["python3", "-m", "py_compile"],
            },
        }

    def test_code_snippet(
        self, code: str, language: str, snippet_id: str
    ) -> Dict[str, Any]:
        """Test a single code snippet"""
        if language not in self.supported_languages:
            return {
                "snippet_id": snippet_id,
                "language": language,
                "status": "error",
                "message": f"Unsupported language: {language}",
            }

        lang_config = self.supported_languages[language]

        # Create temporary file
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=lang_config["extension"], delete=False
        ) as f:
            f.write(code)
            temp_file = f.name

        try:
            # Syntax check first
            syntax_result = self._run_syntax_check(temp_file, language)
            if not syntax_result["success"]:
                return {
                    "snippet_id": snippet_id,
                    "language": language,
                    "status": "syntax_error",
                    "message": syntax_result["error"],
                }

            # Execution test (with timeout)
            exec_result = self._run_execution_test(temp_file, language)

            return {
                "snippet_id": snippet_id,
                "language": language,
                "status": "success" if exec_result["success"] else "execution_error",
                "message": exec_result.get("message", ""),
                "execution_time": exec_result.get("execution_time", 0),
                "output": exec_result.get("output", "")[:500],  # Limit output
            }

        except Exception as e:
            return {
                "snippet_id": snippet_id,
                "language": language,
                "status": "error",
                "message": f"Testing failed: {str(e)}",
            }

        finally:
            # Clean up temporary file
            try:
                os.unlink(temp_file)
            except:
                pass

    def _run_syntax_check(self, file_path: str, language: str) -> Dict[str, Any]:
        """Run syntax check for code"""
        lang_config = self.supported_languages[language]

        try:
            if language == "python" or language == "ros2":
                result = subprocess.run(
                    lang_config["syntax_check"] + [file_path],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                return {
                    "success": result.returncode == 0,
                    "error": result.stderr if result.returncode != 0 else None,
                }

            elif language == "matlab":
                # MATLAB syntax check
                result = subprocess.run(
                    lang_config["syntax_check"]
                    + [f"try, run('{file_path}'); catch ME; disp(ME.message); end;"],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                return {
                    "success": result.returncode == 0
                    and "error" not in result.stdout.lower(),
                    "error": result.stdout
                    if "error" in result.stdout.lower()
                    else None,
                }

        except subprocess.TimeoutExpired:
            return {"success": False, "error": "Syntax check timeout"}
        except Exception as e:
            return {"success": False, "error": f"Syntax check failed: {str(e)}"}

    def _run_execution_test(self, file_path: str, language: str) -> Dict[str, Any]:
        """Run execution test for code"""
        lang_config = self.supported_languages[language]

        try:
            if language == "python" or language == "ros2":
                # For Python, run with timeout and capture output
                result = subprocess.run(
                    lang_config["command"] + [file_path],
                    capture_output=True,
                    text=True,
                    timeout=5,  # 5 second execution limit
                )
                return {
                    "success": result.returncode == 0,
                    "output": result.stdout,
                    "error": result.stderr,
                    "execution_time": 5.0,  # Approximate
                }

            elif language == "matlab":
                # For MATLAB, run in batch mode
                result = subprocess.run(
                    lang_config["command"]
                    + [f"try, run('{file_path}'); catch; end; exit;"],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                return {
                    "success": result.returncode == 0,
                    "output": result.stdout,
                    "error": result.stderr,
                    "execution_time": 10.0,
                }

        except subprocess.TimeoutExpired:
            return {
                "success": False,
                "error": "Execution timeout",
                "execution_time": 5.0,
            }
        except Exception as e:
            return {"success": False, "error": f"Execution failed: {str(e)}"}

    def test_directory(self, directory: Path) -> Dict[str, Any]:
        """Test all code files in a directory"""
        results = []

        # Find code files by language
        for language in self.supported_languages.keys():
            lang_dir = directory / language
            if lang_dir.exists():
                for code_file in lang_dir.glob(
                    f"*{self.supported_languages[language]['extension']}"
                ):
                    # Read code file
                    try:
                        with open(code_file, "r", encoding="utf-8") as f:
                            code = f.read()

                        # Extract snippet ID from filename or content
                        snippet_id = self._extract_snippet_id(code_file, code)

                        # Test the snippet
                        result = self.test_code_snippet(code, language, snippet_id)
                        results.append(result)

                    except Exception as e:
                        results.append(
                            {
                                "snippet_id": str(code_file.name),
                                "language": language,
                                "status": "error",
                                "message": f"Could not read file: {str(e)}",
                            }
                        )

        # Summary statistics
        total = len(results)
        successful = sum(1 for r in results if r["status"] == "success")
        syntax_errors = sum(1 for r in results if r["status"] == "syntax_error")
        execution_errors = sum(1 for r in results if r["status"] == "execution_error")
        other_errors = total - successful - syntax_errors - execution_errors

        return {
            "directory": str(directory),
            "total_snippets": total,
            "successful": successful,
            "syntax_errors": syntax_errors,
            "execution_errors": execution_errors,
            "other_errors": other_errors,
            "success_rate": (successful / total * 100) if total > 0 else 0,
            "results": results,
        }

    def _extract_snippet_id(self, file_path: Path, code: str) -> str:
        """Extract snippet ID from filename or code comments"""
        # Try to get from filename first
        base_name = file_path.stem

        # Try to extract from code comments
        import re

        id_match = re.search(r"#\s*Snippet:\s*(\w+)", code, re.IGNORECASE)
        if id_match:
            return id_match.group(1)

        return base_name


def main():
    if len(sys.argv) != 2:
        print("Usage: python code-tester.py <directory>")
        sys.exit(1)

    directory = Path(sys.argv[1])
    if not directory.exists():
        print(f"Error: Directory {directory} does not exist")
        sys.exit(1)

    tester = CodeTester()
    result = tester.test_directory(directory)

    print(f"\nCode Testing Results for {directory}")
    print("=" * 50)
    print(f"Total snippets: {result['total_snippets']}")
    print(f"Successful: {result['successful']}")
    print(f"Syntax errors: {result['syntax_errors']}")
    print(f"Execution errors: {result['execution_errors']}")
    print(f"Other errors: {result['other_errors']}")
    print(f"Success rate: {result['success_rate']:.1f}%")

    # Show failed tests
    failed_results = [r for r in result["results"] if r["status"] != "success"]
    if failed_results:
        print("\nFAILED TESTS:")
        for failed in failed_results:
            print(f"\n{failed['snippet_id']} ({failed['language']}):")
            print(f"  Status: {failed['status']}")
            print(f"  Message: {failed['message']}")

    # Return appropriate exit code
    sys.exit(
        0 if result["syntax_errors"] == 0 and result["execution_errors"] == 0 else 1
    )


if __name__ == "__main__":
    main()
