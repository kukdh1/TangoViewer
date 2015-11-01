#include <Windows.h>
#include <windowsx.h>
#include <CommCtrl.h>
#include <vector>
#include "Constant.h"
#include "GLWindow.h"
#include "PointCloud.h"
#include "PointCloudMerge.h"
#include "Log.h"
#include "Stopwatch.h"
#include "LUM.h"

#pragma comment(lib, "comctl32")
#pragma comment(linker,"\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
BOOL BrowseFolder(HWND hParent, LPCTSTR szTitle, TCHAR *szFolder);
void OpenGLDraw();
DWORD WINAPI FileOpenThread(LPVOID arg);
DWORD WINAPI ErrorCorrectionThread(LPVOID arg);
DWORD WINAPI LUMThread(LPVOID arg);

LPCWSTR lpszClass = L"Tango Viewer v0.1";

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpszCmdParam, int nCmdShow)
{
	WNDCLASS wndclass;
	MSG msg;
	HWND hWnd;
	RECT rtWindow;
	INITCOMMONCONTROLSEX iccex;

	iccex.dwSize = sizeof(INITCOMMONCONTROLSEX);
	iccex.dwICC = ICC_WIN95_CLASSES;

	if (!InitCommonControlsEx(&iccex))
		return 1;

	wndclass.cbClsExtra = 0;
	wndclass.cbWndExtra = 0;
	wndclass.hbrBackground = (HBRUSH)GetSysColorBrush(COLOR_BTNFACE);
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndclass.hInstance = hInstance;
	wndclass.lpfnWndProc = (WNDPROC)WndProc;
	wndclass.lpszClassName = lpszClass;
	wndclass.lpszMenuName = NULL;
	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	RegisterClass(&wndclass);

	SetRect(&rtWindow, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	AdjustWindowRect(&rtWindow, WS_BORDER | WS_CAPTION | WS_SYSMENU, FALSE);

	hWnd = CreateWindow(lpszClass, lpszClass, WS_BORDER | WS_CAPTION | WS_SYSMENU, CW_USEDEFAULT, CW_USEDEFAULT, rtWindow.right - rtWindow.left, rtWindow.bottom - rtWindow.top, NULL, NULL, hInstance, NULL);
	ShowWindow(hWnd, nCmdShow);

	while (GetMessage(&msg, 0, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	return (int)msg.wParam;
}

kukdh1::GLWindow *glWindow;
kukdh1::LogWindow *lWindow;
HWND hListBox;
HFONT hFont;
WCHAR *pszFolderPath;
HANDLE hKeyboardLock;

kukdh1::Stopwatch stopwatch;

BOOL bDrawFlag[3];

LRESULT CALLBACK WndProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam)
{
	int nCount;
	HANDLE hThread;

	switch (iMessage)
	{
		case WM_CREATE:
			RECT rtGLWindow;
			
			hFont = CreateFont(17, 0, 0, 0, 400, 0, 0, 0, DEFAULT_CHARSET, 3, 2, 1, FF_ROMAN, L"Segoe UI");
			hListBox = CreateWindow(WC_LISTBOX, NULL, WS_VISIBLE | WS_CHILD | LBS_DISABLENOSCROLL| LBS_MULTIPLESEL | LBS_HASSTRINGS | LBS_NOINTEGRALHEIGHT | LBS_NOTIFY, 0, 0, LIST_WIDTH, LIST_HEIGHT, hWnd, (HMENU)ID_LISTBOX, ((LPCREATESTRUCT)lParam)->hInstance, NULL);

			SendMessage(hListBox, WM_SETFONT, (WPARAM)hFont, TRUE);

			SetRect(&rtGLWindow, LIST_WIDTH, 0, LIST_WIDTH + GL_WIDTH, GL_HEIGHT);
			glWindow = new kukdh1::GLWindow(hWnd, ((LPCREATESTRUCT)lParam)->hInstance, &rtGLWindow, OpenGLDraw);
			lWindow = new kukdh1::LogWindow(hWnd, ((LPCREATESTRUCT)lParam)->hInstance);

			lWindow->SetFont(hFont);

			pszFolderPath = (WCHAR *)calloc(MAX_PATH, sizeof(WCHAR));
			if (BrowseFolder(hWnd, L"Choose Folder", pszFolderPath))
			{
				hThread = CreateThread(NULL, 0, FileOpenThread, NULL, NULL, NULL);
				CloseHandle(hThread);
			}
			else
				return -1;

			lWindow->ToggleLogWindow();

			hKeyboardLock = CreateEvent(NULL, TRUE, FALSE, NULL);

			return 0;
		case WM_KEYUP:
			int *nSelected;

			if (WaitForSingleObject(hKeyboardLock, 0) == WAIT_OBJECT_0)
				return 0;

			nCount = SendMessage(hListBox, LB_GETSELCOUNT, 0, 0);
			nSelected = (int *)calloc(nCount, sizeof(int));

			SendMessage(hListBox, LB_GETSELITEMS, nCount, (LPARAM)nSelected);

			switch (wParam)
			{
				case 'S':	//Do Simple Error Correction for selection
					if (nCount == 2)
					{
						lWindow->PrintLog(L"Do Error Correction for %d and %d\r\n", nSelected[0], nSelected[1]);

						stopwatch.tic();
						CalculateCorrection(*(kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, nSelected[0], 0), *(kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, nSelected[1], 0));
						stopwatch.tok();

						lWindow->PrintLog(L"Error Correction Finished (%.3lf)\r\n\r\n", stopwatch.get());
					}
					else
						lWindow->PrintLog(L"Error Correction need two selections of data\r\n");
					break;
				case 'T':	//Generate Plane relation tree and correct error
					hThread = CreateThread(NULL, 0, ErrorCorrectionThread, NULL, NULL, NULL);
					CloseHandle(hThread);

					break;
				case 'U':	//Calculate LUM
					hThread = CreateThread(NULL, 0, LUMThread, NULL, NULL, NULL);
					CloseHandle(hThread);

					break;
				case 'A':	//Save Adjust Matrix for selection
					lWindow->PrintLog(L"Save adjust matrix of selected data\r\n");

					for (int i = 0; i < nCount; i++)
						((kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, nSelected[i], 0))->SaveTransformMatrix();

					lWindow->PrintLog(L"Save adjust matrix Finished\r\n\r\n");
					break;
				case 'E':	//Reset Adjust Matrix for selection
					lWindow->PrintLog(L"Reset adjust matrix of selected data\r\n");

					for (int i = 0; i < nCount; i++)
						((kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, nSelected[i], 0))->ResetTransformMatrix();

					lWindow->PrintLog(L"Reset adjust matrix Finished\r\n\r\n");
					break;
				case 'B':	//Toggle Object Oriented Bounding Box
					bDrawFlag[1] = !bDrawFlag[1];
					glWindow->Refresh();
					break;
				case 'P':	//Toggle Plane OOBB
					bDrawFlag[2] = !bDrawFlag[2];
					glWindow->Refresh();
					break;
				case 'O':	//Toggle Origin
					bDrawFlag[0] = !bDrawFlag[0];
					glWindow->Refresh();
					break;
				case 'R':	//Refresh GLWindow
					glWindow->Refresh();
					break;
				case 'L':	//Toggle LogWindow
					lWindow->ToggleLogWindow();
					break;
				case 'D':	//Deselect All
					SendMessage(hListBox, LB_SETSEL, FALSE, -1);
					glWindow->Refresh();
					break;
			}

			free(nSelected);

			return 0;
		case WM_MOVE:
			RECT rtWindow;

			GetWindowRect(hWnd, &rtWindow);
			lWindow->MoveLogWindow(rtWindow.right, rtWindow.top);

			return 0;
		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
				case ID_LISTBOX:
					switch (HIWORD(wParam))
					{
						case LBN_SELCHANGE:
							glWindow->Refresh();
							SetFocus(hWnd);
							break;
					}
					break;
			}
			return 0;
		case WM_MOUSEWHEEL:
			glWindow->OnWheelMessage(hWnd, wParam, lParam);
			return 0;
		case WM_DESTROY:
			delete glWindow;
			delete lWindow;
			free(pszFolderPath);
			CloseHandle(hKeyboardLock);

			nCount = SendMessage(hListBox, LB_GETCOUNT, 0, 0);
			for (int i = 0; i < nCount; i++)
				delete (kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, i, 0);

			PostQuitMessage(0);

			return 0;
	}

	return DefWindowProc(hWnd, iMessage, wParam, lParam);
}

void OpenGLDraw()
{
	int nCount;
	int *nSelected;

	glPushMatrix();

	nCount = SendMessage(hListBox, LB_GETSELCOUNT, 0, 0);
	nSelected = (int *)calloc(nCount, sizeof(int));

	SendMessage(hListBox, LB_GETSELITEMS, nCount, (LPARAM)nSelected);

	for (int i = 0; i < nCount; i++)
		((kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, nSelected[i], 0))->DrawOnGLWindow(bDrawFlag[0], bDrawFlag[1], bDrawFlag[2]);

	free(nSelected);

	glPopMatrix();
}

DWORD WINAPI FileOpenThread(LPVOID arg)
{
	EnableWindow(hListBox, FALSE);

	WIN32_FIND_DATA wfd;
	WCHAR *pszSearch;
	HANDLE hSearch;
	kukdh1::Stopwatch stopwatch;
	int nIndex;

	int nCount = 0;
	int nPlaneCount;

	lWindow->PrintLog(L"Folder Search Started\r\n");
	stopwatch.tic();

	if (pszFolderPath[wcslen(pszFolderPath) - 1] != L'\\')
		wcscat_s(pszFolderPath, MAX_PATH, L"\\");

	pszSearch = (WCHAR *)calloc(MAX_PATH, sizeof(WCHAR));
	wcscpy_s(pszSearch, MAX_PATH, pszFolderPath);
	wcscat_s(pszSearch, MAX_PATH, L"PointCloud_*.pcdx");

	hSearch = FindFirstFile(pszSearch, &wfd);

	if (hSearch != INVALID_HANDLE_VALUE)
	{
		WCHAR *pszFullPath;

		pszFullPath = (WCHAR *)calloc(MAX_PATH, sizeof(WCHAR));

		do
		{
			if (!(wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			{
				kukdh1::PointCloud *pcTemp;

				wcscpy_s(pszFullPath, MAX_PATH, pszFolderPath);
				wcscat_s(pszFullPath, MAX_PATH, wfd.cFileName);

				pcTemp = new kukdh1::PointCloud();

				if (pcTemp->FromFile(pszFullPath))
				{
					lWindow->PrintLog(L" [%d] %s\r\n", nCount++, wfd.cFileName);

					pcTemp->ApplyStaticalOutlierRemoveFilter();
					nPlaneCount = pcTemp->ExecutePlaneSegmentation();
					pcTemp->CalculateBoundingBox();

					lWindow->PrintLog(L"  %d Planes detected\r\n\r\n", nPlaneCount);

					nIndex = SendMessage(hListBox, LB_ADDSTRING, -1, (LPARAM)wfd.cFileName);
					SendMessage(hListBox, LB_SETITEMDATA, nIndex, (LPARAM)pcTemp);
				}
				else
					delete pcTemp;
			}
		} while (FindNextFile(hSearch, &wfd));

		FindClose(hSearch);
		free(pszFullPath);

		SendMessage(hListBox, LB_SETSEL, TRUE, -1);
	}

	EnableWindow(hListBox, TRUE);
	glWindow->Refresh();

	stopwatch.tok();
	lWindow->PrintLog(L"%d File Found and Read (%.3lf ms)\r\n\r\n", nCount, stopwatch.get());

	return 0;
}

DWORD WINAPI ErrorCorrectionThread(LPVOID arg)
{
	size_t stCloudCount;
	kukdh1::PointCloudVector vCloudData;

	EnableWindow(hListBox, FALSE);
	SetEvent(hKeyboardLock);

	stCloudCount = SendMessage(hListBox, LB_GETCOUNT, 0, 0);

	lWindow->PrintLog(L"Generate Relation Tree for %d planes\r\n", stCloudCount);
	stopwatch.tic();

	for (size_t i = 0; i < stCloudCount; i++)
		vCloudData.push_back((kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, i, 0));

	{
		kukdh1::PlaneTree ptTree;

		ptTree.AddPointCloud(vCloudData);
		ptTree.DoErrorCorrection(lWindow);
	}

	stopwatch.tok();

	glWindow->Refresh();

	vCloudData.clear();

	lWindow->PrintLog(L"End of error correction (%.3lfms)\r\n\r\n", stopwatch.get());

	ResetEvent(hKeyboardLock);
	EnableWindow(hListBox, TRUE);

	return 0;
}

DWORD WINAPI LUMThread(LPVOID arg)
{
	size_t stCloudCount;
	kukdh1::PointCloudVector vCloudData;

	EnableWindow(hListBox, FALSE);
	SetEvent(hKeyboardLock);

	stCloudCount = SendMessage(hListBox, LB_GETCOUNT, 0, 0);

	lWindow->PrintLog(L"Calculate LUM for %d planes\r\n", stCloudCount);
	stopwatch.tic();

	for (size_t i = 0; i < stCloudCount; i++)
		vCloudData.push_back((kukdh1::PointCloud *)SendMessage(hListBox, LB_GETITEMDATA, i, 0));

	{
		kukdh1::LUM lLUM;

		lLUM.AddPointClouds(vCloudData);
		lLUM.Compute();
	}

	stopwatch.tok();

	glWindow->Refresh();

	vCloudData.clear();

	lWindow->PrintLog(L"End of calculation (%.3lfms)\r\n\r\n", stopwatch.get());

	ResetEvent(hKeyboardLock);
	EnableWindow(hListBox, TRUE);

	return 0;
}

#include <shlobj.h>

BOOL BrowseFolder(HWND hParent, LPCTSTR szTitle, TCHAR *szFolder)
{
	LPMALLOC pMalloc;
	LPITEMIDLIST pidl;
	BROWSEINFO bi;

	memset(&bi, 0, sizeof(BROWSEINFO));
	bi.hwndOwner = hParent;
	bi.pidlRoot = NULL;
	bi.pszDisplayName = NULL;
	bi.lpszTitle = szTitle;
	bi.ulFlags = 0;

	pidl = SHBrowseForFolder(&bi);

	if (pidl == NULL) {
		return FALSE;
	}
	SHGetPathFromIDList(pidl, szFolder);

	if (SHGetMalloc(&pMalloc) != NOERROR) {
		return FALSE;
	}
	pMalloc->Free(pidl);
	pMalloc->Release();
	return TRUE;
}