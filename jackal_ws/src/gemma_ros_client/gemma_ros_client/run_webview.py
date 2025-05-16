import webview
import sys
import os

# main 함수만 정의
def main():
    if len(sys.argv) < 2:
        sys.exit(1)

    html_file_path = sys.argv[1]

    url = f"file://{os.path.abspath(html_file_path)}"

    webview.create_window('Agricsense', url=url)
    webview.start()

if __name__ == '__main__':
    main()