# Contributing to Isaac Sim Learning Environment

Thank you for your interest in contributing to this Isaac Sim learning environment! This document provides guidelines and information for contributors.

## 🤝 How to Contribute

### Types of Contributions

We welcome several types of contributions:

1. **Bug Reports** - Report issues you encounter
2. **Feature Requests** - Suggest new features or improvements
3. **Documentation** - Improve or add documentation
4. **Code Examples** - Add new examples or tutorials
5. **Tutorials** - Create step-by-step guides
6. **Assets** - Share 3D models, environments, or textures

### Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/nvidia-issac-sim.git
   cd nvidia-issac-sim
   ```
3. **Create a new branch** for your changes:
   ```bash
   git checkout -b feature/your-feature-name
   ```
4. **Make your changes** and test them
5. **Commit your changes** with clear messages
6. **Push to your fork** and create a Pull Request

## 📝 Development Guidelines

### Code Style

#### Python Code
- Follow PEP 8 style guidelines
- Use meaningful variable and function names
- Add docstrings to functions and classes
- Keep functions small and focused
- Use type hints where appropriate

#### Bash Scripts
- Use consistent indentation (2 spaces)
- Add comments explaining complex logic
- Use meaningful variable names
- Check for errors and handle them gracefully

#### Documentation
- Use clear, concise language
- Include code examples where helpful
- Keep documentation up to date
- Use proper markdown formatting

### File Organization

#### Adding New Examples
- Place examples in the appropriate `13-examples/` subdirectory
- Include a README.md explaining the example
- Add proper error handling and comments
- Test the example before submitting

#### Adding New Tutorials
- Place tutorials in `14-tutorials/` with appropriate subdirectory
- Include step-by-step instructions
- Add screenshots or diagrams where helpful
- Test the tutorial on a clean environment

#### Adding New Assets
- Place assets in `12-assets/` with appropriate subdirectory
- Include metadata about the asset
- Ensure assets are properly licensed
- Compress large files appropriately

### Commit Messages

Use clear, descriptive commit messages:

```
feat: add new robot movement example
fix: correct ROS2 bridge configuration
docs: update setup guide for macOS
style: format Python code according to PEP 8
refactor: reorganize tutorial structure
test: add unit tests for robot controller
```

## 🧪 Testing

### Before Submitting

1. **Test your changes** thoroughly
2. **Run the CI checks** locally:
   ```bash
   # Check Python code style
   flake8 .
   black --check .
   isort --check-only .
   
   # Check script syntax
   bash -n 11-scripts/*.sh
   bash -n 09-docker-setup/*.sh
   ```
3. **Test Docker setup** if you modified it:
   ```bash
   cd 09-docker-setup
   docker-compose config
   ```
4. **Verify documentation** links and formatting

### Testing Guidelines

- Test on clean environments when possible
- Verify examples work as described
- Check that tutorials are complete and accurate
- Ensure scripts handle errors gracefully

## 📋 Pull Request Process

### Before Creating a PR

1. **Update documentation** if your changes affect it
2. **Add tests** for new functionality
3. **Update README** if you add new features
4. **Check that CI passes** on your branch

### PR Description

Include the following in your PR description:

1. **Summary** of changes
2. **Type of change** (bug fix, feature, documentation, etc.)
3. **Testing** performed
4. **Screenshots** if applicable
5. **Breaking changes** if any

### PR Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Code refactoring
- [ ] Performance improvement

## Testing
- [ ] Tested locally
- [ ] All examples work
- [ ] Documentation updated
- [ ] CI passes

## Screenshots (if applicable)
Add screenshots here

## Additional Notes
Any additional information
```

## 🐛 Bug Reports

When reporting bugs, please include:

1. **Clear description** of the issue
2. **Steps to reproduce** the problem
3. **Expected behavior** vs actual behavior
4. **System information** (OS, Isaac Sim version, etc.)
5. **Error messages** or logs
6. **Screenshots** if applicable

### Bug Report Template

```markdown
## Bug Description
Clear description of the bug

## Steps to Reproduce
1. Step one
2. Step two
3. Step three

## Expected Behavior
What should happen

## Actual Behavior
What actually happens

## System Information
- OS: [e.g., macOS 12.0]
- Isaac Sim Version: [e.g., 2022.1.1]
- Docker Version: [e.g., 20.10.8]
- Python Version: [e.g., 3.9.7]

## Error Messages
```
Paste error messages here
```

## Additional Context
Any other relevant information
```

## 💡 Feature Requests

When requesting features, please include:

1. **Clear description** of the feature
2. **Use case** and motivation
3. **Proposed implementation** (if you have ideas)
4. **Alternatives considered**
5. **Additional context**

## 📚 Documentation

### Writing Guidelines

- Use clear, simple language
- Include code examples
- Add screenshots for complex procedures
- Keep tutorials step-by-step
- Update related documentation when making changes

### Documentation Structure

- **README.md** - Project overview and quick start
- **QUICK_START.md** - 5-minute setup guide
- **CONTRIBUTING.md** - This file
- **Directory READMEs** - Specific to each section
- **Tutorials** - Step-by-step guides
- **Examples** - Code with explanations

## 🏷️ Issue Labels

We use the following labels for issues:

- `bug` - Something isn't working
- `enhancement` - New feature or request
- `documentation` - Improvements or additions to documentation
- `good first issue` - Good for newcomers
- `help wanted` - Extra attention is needed
- `question` - Further information is requested
- `wontfix` - This will not be worked on

## 📞 Getting Help

If you need help:

1. **Check existing issues** and discussions
2. **Read the documentation** thoroughly
3. **Ask questions** in GitHub Discussions
4. **Join the community** forums

## 📄 License

By contributing to this project, you agree that your contributions will be licensed under the same license as the project.

## 🙏 Recognition

Contributors will be recognized in:
- CONTRIBUTORS.md file
- Release notes
- Project documentation

Thank you for contributing to the Isaac Sim learning community! 🚀
