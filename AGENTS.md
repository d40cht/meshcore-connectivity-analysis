# AGENT GUIDELINES: Rust Coding Standards

This document defines the development standards and workflow for the implementation of this project. All contributors (human or AI) must adhere to these constraints.

## 1. Tooling & Environment
* **Language:** Rust (Latest Stable).
* **Build System:** `cargo`.
* **Dependency Management:** * Do **NOT** manually edit `Cargo.toml` to guess or update package versions. 
    * Use `cargo add <crate_name>` to add or update dependencies.
* **Formatting:** All code must be formatted using `cargo fmt` before submission.
* **Linting:** * Code must compile with **zero warnings**. 
    * Use `cargo check` to ensure code quality. 
    * Submissions with warnings will be rejected.

## 2. Quality & Testing
* **Tests Required:** Every core logic component, mathematical utility, and data parser must have associated unit tests.
* **Functionality:** All PRs must pass `cargo test`.
* **Reliability:** Handle errors gracefully. Avoid `unwrap()` or `expect()` in core logic unless it is mathematically proven to be safe. Use `Result` and `Option` types.

## 3. Coding Style
* **Explicitness:** Prefer clear, descriptive variable names and include doc-comments (`///`) for public structures and functions.
* **Comment style:** No end of line comments please, and no ephemeral comments describing decisions just made or changes just in one PR. They belong in the PR comments not permanently in the codebase.

## 4. Submission Checklist
Before finalizing a PR or sending out code, the following commands must be run and must succeed:
1. `cargo fmt`
2. `cargo check` (0 warnings)
3. `cargo test` (All tests must pass)
