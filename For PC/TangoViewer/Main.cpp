#include <Windows.h>
#include <CommCtrl.h>
#include <vector>
#include "GLWindow.h"
#include "PointCloud.h"

#pragma comment(lib, "comctl32")

#define WINDOW_WIDTH	1400
#define WINDOW_HEIGHT	1000
#define LIST_WIDTH		400
#define LIST_HEIGHT		WINDOW_HEIGHT
#define GL_WIDTH		1000
#define GL_HEIGHT		WINDOW_HEIGHT

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
BOOL BrowseFolder(HWND hParent, LPCTSTR szTitle, TCHAR *szFolder);
void OpenGLDraw();

LPCWSTR lpszClass = L"Tango Viewer v0.0";

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpszCmdParam, int nCmdShow)
{
	WNDCLASS wndclass;
	MSG msg;
	HWND hWnd;
	RECT rtWindow;

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
std::vector<kukdh1::PointCloud *> vPointClouds;
WCHAR *pszFolderPath;

LRESULT CALLBACK WndProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam)
{
	switch (iMessage)
	{
		case WM_CREATE:
			RECT rtGLWindow;

			SetRect(&rtGLWindow, LIST_WIDTH, 0, LIST_WIDTH + GL_WIDTH, GL_HEIGHT);
			glWindow = new kukdh1::GLWindow(hWnd, ((LPCREATESTRUCT)lParam)->hInstance, &rtGLWindow, OpenGLDraw);

			pszFolderPath = (WCHAR *)calloc(MAX_PATH, sizeof(WCHAR));
			if (BrowseFolder(hWnd, L"Choose Folder", pszFolderPath))
			{
				WIN32_FIND_DATA wfd;
				WCHAR *pszSearch;
				HANDLE hSearch;

				if (pszFolderPath[wcslen(pszFolderPath) - 1] != L'\\')
					wcscat_s(pszFolderPath, MAX_PATH, L"\\");

				pszSearch = (WCHAR *)calloc(MAX_PATH, sizeof(WCHAR));
				wcscpy_s(pszSearch, MAX_PATH, pszFolderPath);
				wcscat_s(pszSearch, MAX_PATH, L"PointCloud_*.bin");

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
							pcTemp->FromFile(pszFullPath);

							vPointClouds.push_back(pcTemp);
						}
					} while (FindNextFile(hSearch, &wfd));

					free(pszFullPath);
				}
			}

			return 0;
		case WM_MOUSEWHEEL:
			glWindow->OnWheelMessage(hWnd, wParam, lParam);
			return 0;
		case WM_DESTROY:
			delete glWindow;
			free(pszFolderPath);

			for (unsigned int i = 0; i < vPointClouds.size(); i++)
				delete vPointClouds.at(i);

			PostQuitMessage(0);
			return 0;
	}

	return DefWindowProc(hWnd, iMessage, wParam, lParam);
}

void OpenGLDraw()
{
	glPushMatrix();

	for (size_t i = 0; i < vPointClouds.size(); i++)
		vPointClouds.at(i)->DrawOnGLWindow();

	glPopMatrix();
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