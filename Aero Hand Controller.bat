@echo off
pip show aero-open-sdk >nul 2>&1
if errorlevel 1 (
    echo First run: installing Aero Hand SDK...
    pip install aero-open-sdk
    if errorlevel 1 (
        echo.
        echo ERROR: Could not install SDK.
        echo Make sure Python is installed from https://python.org
        echo During install, tick "Add Python to PATH"
        pause
        exit /b 1
    )
    echo.
)
python "%~dp0aero_hand_gui.py"
if errorlevel 1 (
    echo.
    echo ERROR: See message above.
    pause
)
