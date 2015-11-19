#include "Log.h"

namespace kukdh1
{
	LogWindow::LogWindow(HWND hParent, HINSTANCE hInstance)
	{
		WNDCLASS wndclass;

		wndclass.cbClsExtra = 0;
		wndclass.cbWndExtra = 0;
		wndclass.hbrBackground = (HBRUSH)GetSysColorBrush(COLOR_BTNFACE);
		wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
		wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wndclass.hInstance = hInstance;
		wndclass.lpfnWndProc = (WNDPROC)LogProc;
		wndclass.lpszMenuName = NULL;
		wndclass.lpszClassName = LOG_CLASS_NAME;
		wndclass.style = CS_HREDRAW | CS_VREDRAW;

		RegisterClass(&wndclass);

		this->hParent = hParent;

		hWnd = CreateWindow(LOG_CLASS_NAME, LOG_CLASS_NAME, WS_SYSMENU | WS_CAPTION | WS_BORDER, 0, 0, 0, 0, hParent, NULL, hInstance, this);
	}

	void LogWindow::PrintLog(LPCWSTR lpszFormat, ...)
	{
		va_list args;
		LPWSTR lpszString;

		lpszString = (LPWSTR)calloc(TEXT_BUFFER_SIZE, sizeof(WCHAR));

		va_start(args, lpszFormat);
		vswprintf_s(lpszString, TEXT_BUFFER_SIZE, lpszFormat, args);
		va_end(args);

		//Move caret to end
		int nLength;

		nLength = GetWindowTextLength(hEdit);
		SendMessage(hEdit, EM_SETSEL, nLength, nLength);

		//Append Text
		SendMessage(hEdit, EM_REPLACESEL, FALSE, (LPARAM)lpszString);

		//Check line count
		int nLine;

		nLine = SendMessage(hEdit, EM_GETLINECOUNT, 0, 0);

		if (nLine > MAX_LOG_LINE)
		{
			//Get character index of line
			nLine = SendMessage(hEdit, EM_LINEINDEX, nLine - MAX_LOG_LINE, 0);
			
			//Select lines to delete
			SendMessage(hEdit, EM_SETSEL, 0, nLine);

			//Delete lines
			SendMessage(hEdit, EM_REPLACESEL, FALSE, (LPARAM)L"");
		}

		nLength = GetWindowTextLength(hEdit);
		SendMessage(hEdit, EM_SETSEL, nLength, nLength);
		SendMessage(hEdit, EM_SCROLLCARET, 0, 0);
		
		free(lpszString);
	}

	void LogWindow::SetFont(HFONT hFont)
	{
		SendMessage(hEdit, WM_SETFONT, (WPARAM)hFont, TRUE);
	}

	void LogWindow::ToggleLogWindow()
	{
		SendMessage(hWnd, WM_LOG_SHOW, 0, 0);
	}

	void LogWindow::MoveLogWindow(short sNewX, short sNewY)
	{
		MoveWindow(hWnd, sNewX, sNewY, LOG_WIDTH, rtWindow.bottom - rtWindow.top, TRUE);
	}

	LRESULT LogWindow::LogProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam)
	{
		LogWindow * plwThis;

		switch (iMessage)
		{
			case WM_CREATE:
				RECT rtParent;

				SetClassObject(hWnd, (LogWindow *)((LPCREATESTRUCT)lParam)->lpCreateParams);
				plwThis = GetClassObject(hWnd);

				GetWindowRect(plwThis->hParent, &rtParent);

				SetRect(&plwThis->rtWindow, rtParent.right, rtParent.top, rtParent.right + LOG_WIDTH, rtParent.bottom);
				MoveWindow(hWnd, plwThis->rtWindow.left, plwThis->rtWindow.top, plwThis->rtWindow.right - plwThis->rtWindow.left, plwThis->rtWindow.bottom - plwThis->rtWindow.top, FALSE);

				plwThis->hEdit = CreateWindow(L"Edit", NULL, WS_CHILD | WS_VISIBLE | ES_READONLY | ES_MULTILINE | WS_VSCROLL, 0, 0, 0, 0, hWnd, (HMENU)ID_LOG_EDIT, ((LPCREATESTRUCT)lParam)->hInstance, NULL);
	
				return 0;
			case WM_SIZE:
				plwThis = GetClassObject(hWnd);

				if (plwThis)
				{
					MoveWindow(plwThis->hEdit, 0, 0, LOWORD(lParam), HIWORD(lParam), TRUE);
				}
				return 0;
			case WM_LOG_SHOW:
				if (IsWindowVisible(hWnd))
					ShowWindow(hWnd, SW_HIDE);
				else
					ShowWindow(hWnd, SW_SHOWNOACTIVATE);

				return 0;
			case WM_CLOSE:
				ShowWindow(hWnd, SW_HIDE);
				return 0;
		}

		return DefWindowProc(hWnd, iMessage, wParam, lParam);
	}

	LogWindow * LogWindow::GetClassObject(HWND hWnd)
	{
		return (LogWindow *)GetWindowLongPtr(hWnd, GWLP_USERDATA);
	}

	LogWindow * LogWindow::SetClassObject(HWND hWnd, LogWindow *plwNew)
	{
		return (LogWindow *)SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)plwNew);
	}
}