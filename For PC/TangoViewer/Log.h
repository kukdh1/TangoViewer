#pragma once

#ifndef _LOG_H_
#define _LOG_H_

#include <Windows.h>
#include <stdio.h>
#include "Constant.h"

namespace kukdh1
{
	class LogWindow
	{
		private:
			HWND hWnd;
			HWND hParent;
			RECT rtWindow;

			HWND hEdit;

			static LogWindow * GetClassObject(HWND hWnd);
			static LogWindow * SetClassObject(HWND hWnd, LogWindow *plwNew);
		public:
			LogWindow(HWND hParent, HINSTANCE hInstance);

			void PrintLog(LPCWSTR lpszFormat, ...);
			
			void ToggleLogWindow();
			void MoveLogWindow(short sNewX, short sNewY);
			void SetFont(HFONT hFont);

			static LRESULT CALLBACK LogProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam);
	};
}

#endif