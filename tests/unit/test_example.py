"""
Example Unit Test

Unit tests should be:
- Hermetic (no external dependencies)
- Fast (< 100ms each)
- Isolated (no side effects)
- Deterministic (same result every time)
"""

import pytest


@pytest.mark.unit
def test_example_pass():
    """Example passing test"""
    assert 1 + 1 == 2


@pytest.mark.unit
def test_example_with_fixture(repo_root):
    """Example test using fixture"""
    assert repo_root.exists()
    assert (repo_root / "setup.sh").exists()


@pytest.mark.unit
class TestExampleClass:
    """Example test class"""
    
    def test_method_1(self):
        """Test method 1"""
        assert True
    
    def test_method_2(self):
        """Test method 2"""
        result = self._helper_function(5)
        assert result == 10
    
    @staticmethod
    def _helper_function(x: int) -> int:
        """Helper function"""
        return x * 2


@pytest.mark.unit
@pytest.mark.parametrize("input,expected", [
    (0, 0),
    (1, 2),
    (5, 10),
    (-3, -6),
])
def test_example_parametrized(input: int, expected: int):
    """Example parametrized test"""
    result = input * 2
    assert result == expected
