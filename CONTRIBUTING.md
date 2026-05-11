# Contributing to Just Physics Engine

Thank you for your interest in contributing to **just_physics_engine**. We welcome all contributions: bug fixes, features, documentation improvements, and tests.

---

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Suggesting Features](#suggesting-features)
  - [Submitting a Pull Request](#submitting-a-pull-request)
- [Development Setup](#development-setup)
- [Coding Guidelines](#coding-guidelines)
- [Commit Message Convention](#commit-message-convention)
- [License](#license)

---

## Code of Conduct

This project follows our [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you agree to abide by its terms. Please report unacceptable behavior to the maintainers.

---

## Getting Started

1. **Fork** the repository on GitHub.
2. **Clone** your fork locally:
   ```bash
   git clone https://github.com/just-unknown-dev/just-physics-engine.git
   cd just-physics-engine
   ```
4. **Install dependencies:**
   ```bash
   flutter pub get
   ```
5. **Run tests and analysis** to make sure everything is green before you start:
   ```bash
   flutter test
   flutter analyze
   ```

---

## How to Contribute

### Reporting Bugs

Before opening an issue, please:
- Search [existing issues](../../issues) to avoid duplicates.
- Confirm the bug is reproducible on the latest version.

When opening a bug report, include:
- A clear, descriptive title.
- Steps to reproduce the problem.
- Expected vs. actual behavior.
- Flutter/Dart SDK versions (`flutter --version`).
- A minimal code sample or link to a reproduction repo if possible.

### Suggesting Features

- Open a [GitHub Discussion](../../discussions) or issue labeled **`enhancement`**.
- Describe the problem your feature solves and how you'd like it to behave.
- Check that the feature aligns with the package scope (physics simulation for Flutter games).

### Submitting a Pull Request

1. Create a topic branch from `main`:
   ```bash
   git checkout -b feat/my-new-feature
   ```
2. Make your changes, following the [Coding Guidelines](#coding-guidelines).
3. Add or update tests for any changed behavior.
4. Ensure all tests pass:
   ```bash
   flutter test
   flutter analyze
   ```
5. Commit with a [conventional commit message](#commit-message-convention).
6. Push to your fork and open a Pull Request against `main`.
7. Fill in the PR template — describe what changed and why.
8. Address any review feedback promptly.

> **Small PRs are easier to review.** If your change is large, consider opening an issue first to discuss the approach.

---

## Development Setup

| Tool | Minimum Version | Recommended Version |
|------|------------------|---------------------|
| Flutter | 1.17.0 | latest stable |
| Dart SDK | 3.11.0 | 3.11.x |

This package lives in the monorepo at `packages/just_physics_engine`.

```bash
# From the repository root
cd packages/just_physics_engine

# Fetch dependencies
flutter pub get

# Run tests
flutter test

# Analyze
flutter analyze
```

---

## Coding Guidelines

- Follow the official [Dart style guide](https://dart.dev/guides/language/effective-dart/style).
- All public APIs must have **doc comments** (`///`).
- Prefer `const` constructors wherever possible.
- Do not introduce new dependencies without prior discussion in an issue.
- Keep modules decoupled; changes to collision, integration, broad-phase, and helpers should remain testable in isolation.
- Match the existing file and folder structure under `lib/src/`.
- Prefer deterministic update behavior and avoid introducing frame-rate-dependent physics changes.

---

## Commit Message Convention

We use [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short summary>
```

| Type | When to use |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `refactor` | Code change that neither fixes a bug nor adds a feature |
| `test` | Adding or updating tests |
| `chore` | Build process, tooling, dependencies |

**Examples:**
```
feat(physics): add mass-proportional collision resolution
fix(physics): correct positional correction on stacked contacts
docs(contributing): add development setup section
```

---

## License

By contributing to this repository, you agree that your contributions will be licensed under the [BSD-3-Clause License](LICENSE).
