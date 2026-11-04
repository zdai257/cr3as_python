"""
Run all tests
"""

import os
import sys
import subprocess

# Add SDK path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def run_test(test_file):
  """Run one test file"""
  print(f"\n{'='*60}")
  print(f"Running: {test_file}")
  print('='*60)
  
  try:
    result = subprocess.run(
      [sys.executable, test_file],
      capture_output=True,
      text=True,
      cwd=os.path.dirname(__file__)
    )
    
    print(result.stdout)
    
    if result.stderr:
      print("stderr:")
      print(result.stderr)
    
    return result.returncode == 0
  except Exception as e:
    print(f"Test run failed: {e}")
    return False


def main():
  """Main"""
  print("="*60)
  print("Dobot SDK V4 - test suite")
  print("="*60)
  
  test_files = [
    'test_basic.py',
    'test_io.py',
    'test_motion.py',
    'test_robot_control.py',
    'test_plugins.py',
    'test_communication.py'
  ]
  
  passed = 0
  failed = 0
  
  for test_file in test_files:
    file_path = os.path.join(os.path.dirname(__file__), test_file)
    
    if os.path.exists(file_path):
      if run_test(file_path):
        passed += 1
      else:
        failed += 1
    else:
      print(f"\nWarning: missing test file - {test_file}")
      failed += 1
  
  # Summary
  print("\n" + "="*60)
  print("Test summary")
  print("="*60)
  print(f"passed: {passed}")
  print(f"failed: {failed}")
  print(f"total: {passed + failed}")
  
  if failed == 0:
    print("\n🎉 All tests passed")
    return 0
  else:
    print(f"\n❌ {failed} tests failed")
    return 1


if __name__ == "__main__":
  sys.exit(main())
