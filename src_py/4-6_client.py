import socket
import threading
import queue
import tkinter as tk
from tkinter import messagebox
from datetime import datetime


# ============================================================
# 서버 설정
# ============================================================

PORT = 5000

sock = None
connected = False
logged_in = False
current_user = None
response_queue = queue.Queue()


# ============================================================
# 통신
# ============================================================

def send_request(action, content):
    global sock

    if sock is None:
        return None

    try:
        sock.sendall(
            f"{action} : {content}".encode("utf-8")
        )

        while True:
            response = response_queue.get()
            if response is None:
                return None

            if handle_server_disconnect_message(response):
                return None

            return response

    except Exception as e:
        messagebox.showerror("통신 오류", str(e))
        return None


def parse_response(response):

    if response is None:
        return None, None

    if " : " not in response:
        return None, None

    action, content = response.split(
        " : ",
        1
    )

    return action, content


def handle_server_disconnect_message(response):
    global sock
    global connected
    global logged_in
    global current_user

    action, content = parse_response(response)

    if action != "접속종료":
        return False

    try:
        if sock:
            sock.close()
    except Exception:
        pass

    sock = None
    connected = False
    logged_in = False
    current_user = None

    message = content
    if message.startswith("TIMEOUT|"):
        message = message.split("|", 1)[1]

    def show_disconnect():
        messagebox.showinfo("자동 접속 해제", message)
        show_connect_screen()

    root.after(0, show_disconnect)
    return True


def server_receiver():
    global sock

    while True:
        current_sock = sock
        if current_sock is None:
            return

        try:
            current_sock.settimeout(1)
            data = current_sock.recv(4096)

            if not data:
                return

            response = data.decode("utf-8")

            if handle_server_disconnect_message(response):
                return

            response_queue.put(response)

        except socket.timeout:
            continue
        except Exception:
            return


def start_server_receiver():
    threading.Thread(target=server_receiver, daemon=True).start()


# ============================================================
# 날짜 표시
# ============================================================

def update_date(label):

    today = datetime.now().strftime(
        "%Y-%m-%d"
    )

    label.config(
        text=today
    )

    label.after(
        1000,
        lambda: update_date(label)
    )


# ============================================================
# 공통 화면
# ============================================================

def clear_screen():

    for widget in root.winfo_children():
        widget.destroy()


def create_date_header():

    date_label = tk.Label(
        root,
        text="",
        font=("Arial", 13, "bold")
    )

    date_label.pack(
        pady=(10, 5)
    )

    update_date(date_label)


# ============================================================
# 접속 화면
# ============================================================

def show_connect_screen():

    clear_screen()

    create_date_header()

    title = tk.Label(
        root,
        text="서버 접속",
        font=("Arial", 22, "bold")
    )

    title.pack(pady=20)

    ip_label = tk.Label(
        root,
        text="서버 IP"
    )

    ip_label.pack()

    ip_entry = tk.Entry(
        root,
        width=30,
        font=("Arial", 14)
    )

    ip_entry.pack(pady=10)

    # 기본값
    ip_entry.insert(
        0,
        "127.0.0.1"
    )

    def connect():

        global sock
        global connected

        ip = ip_entry.get().strip()

        if not ip:
            messagebox.showwarning(
                "입력 오류",
                "서버 IP를 입력하세요."
            )

            return

        try:

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            sock.connect(
                (ip, PORT)
            )

            start_server_receiver()

            response = send_request(
                "접속",
                "접속 요청"
            )

            action, content = parse_response(
                response
            )

            if (
                action == "접속"
                and content.startswith("OK|")
            ):

                connected = True

                messagebox.showinfo(
                    "접속",
                    "서버에 접속했습니다."
                )

                show_login_screen()

            else:

                messagebox.showerror(
                    "접속 실패",
                    content.replace(
                        "FAIL|",
                        ""
                    )
                    if content
                    else "접속 실패"
                )

                sock.close()
                sock = None

        except Exception as e:

            messagebox.showerror(
                "접속 실패",
                str(e)
            )

            sock = None


    connect_button = tk.Button(
        root,
        text="서버 접속",
        width=20,
        font=("Arial", 13),
        command=connect
    )

    connect_button.pack(
        pady=20
    )


# ============================================================
# 로그인 화면
# ============================================================

def show_login_screen():

    clear_screen()

    create_date_header()

    title = tk.Label(
        root,
        text="로그인",
        font=("Arial", 22, "bold")
    )

    title.pack(pady=20)


    # ID
    id_label = tk.Label(
        root,
        text="ID"
    )

    id_label.pack()

    id_entry = tk.Entry(
        root,
        width=30,
        font=("Arial", 14)
    )

    id_entry.pack(pady=2)

    tk.Label(
        root,
        text="※ 영문 소문자(a-z) + 숫자(0-9)만 허용 / 1~10자 / 띄어쓰기 불가",
        font=("Arial", 9)
    ).pack()

    # PW
    pw_label = tk.Label(
        root,
        text="PW"
    )

    pw_label.pack()

    pw_entry = tk.Entry(
        root,
        width=30,
        font=("Arial", 14),
        show="*"
    )

    pw_entry.pack(pady=5)


    def login():

        global logged_in
        global current_user

        user_id = id_entry.get()
        password = pw_entry.get()

        if not user_id or not password:

            messagebox.showwarning(
                "입력 오류",
                "ID와 PW를 모두 입력하세요."
            )

            return

        response = send_request(
            "로그인",
            f"{user_id}|{password}"
        )

        action, content = parse_response(
            response
        )

        if action != "로그인":
            return

        if content.startswith("OK|"):

            parts = content.split("|")

            current_user = parts[1]

            logged_in = True

            messagebox.showinfo(
                "로그인",
                "로그인 성공"
            )

            show_work_screen()

        else:

            messagebox.showerror(
                "로그인 실패",
                content.replace(
                    "FAIL|",
                    ""
                )
            )


    login_button = tk.Button(
        root,
        text="로그인",
        width=20,
        font=("Arial", 13),
        command=login
    )

    login_button.pack(pady=15)


    signup_button = tk.Button(
        root,
        text="회원가입",
        width=20,
        font=("Arial", 13),
        command=show_signup_screen
    )

    signup_button.pack(pady=5)


    def disconnect():
        global sock
        global connected
        global logged_in
        global current_user

        try:
            if sock:
                sock.close()
        except Exception:
            pass

        sock = None
        connected = False
        logged_in = False
        current_user = None

        # 접속 해제 후 이전 페이지(서버 접속 화면)로 이동
        show_connect_screen()


    disconnect_button = tk.Button(
        root,
        text="접속 해제",
        width=20,
        font=("Arial", 13),
        command=disconnect
    )

    disconnect_button.pack(pady=5)


# ============================================================
# 회원가입 화면
# ============================================================

def show_signup_screen():

    clear_screen()

    create_date_header()

    title = tk.Label(
        root,
        text="회원가입",
        font=("Arial", 22, "bold")
    )

    title.pack(pady=20)


    # ID
    tk.Label(
        root,
        text="ID"
    ).pack()

    id_entry = tk.Entry(
        root,
        width=30,
        font=("Arial", 14)
    )

    id_entry.pack(pady=2)

    tk.Label(
        root,
        text="※ 영문 소문자(a-z) + 숫자(0-9)만 허용 / 1~10자 / 띄어쓰기 불가",
        font=("Arial", 9)
    ).pack()

    # PW
    tk.Label(
        root,
        text="PW"
    ).pack()

    pw_entry = tk.Entry(
        root,
        width=30,
        font=("Arial", 14),
        show="*"
    )

    pw_entry.pack(pady=2)

    tk.Label(
        root,
        text="※ 영문 소문자(a-z) + 숫자(0-9)만 허용 / 1~10자 / 띄어쓰기 불가",
        font=("Arial", 9)
    ).pack()

    # 사용자명
    tk.Label(
        root,
        text="사용자명"
    ).pack()

    name_entry = tk.Entry(
        root,
        width=30,
        font=("Arial", 14)
    )

    name_entry.pack(pady=2)

    tk.Label(
        root,
        text="※ 한글 또는 영문만 허용 / 1~10자 / 띄어쓰기 불가",
        font=("Arial", 9)
    ).pack()

    def signup():

        user_id = id_entry.get().strip()
        password = pw_entry.get()
        user_name = name_entry.get().strip()

        if not user_id or not password or not user_name:

            messagebox.showwarning(
                "입력 오류",
                "모든 항목을 입력하세요."
            )

            return

        if not (
            1 <= len(user_id) <= 10
            and all(("a" <= ch <= "z") or ("0" <= ch <= "9") for ch in user_id)
        ):
            messagebox.showwarning(
                "입력 오류",
                "ID는 영문 소문자(a-z)와 숫자(0-9)만 사용하여 1~10자로 입력하세요."
            )
            return

        if not (
            1 <= len(password) <= 10
            and all(("a" <= ch <= "z") or ("0" <= ch <= "9") for ch in password)
        ):
            messagebox.showwarning(
                "입력 오류",
                "PW는 영문 소문자(a-z)와 숫자(0-9)만 사용하여 1~10자로 입력하세요."
            )
            return

        if not (
            1 <= len(user_name) <= 10
            and all(
                ("가" <= ch <= "힣")
                or ("A" <= ch <= "Z")
                or ("a" <= ch <= "z")
                for ch in user_name
            )
        ):
            messagebox.showwarning(
                "입력 오류",
                "사용자명은 한글 또는 영문만 사용하여 1~10자로 입력하세요."
            )
            return

        response = send_request(
            "회원가입",
            f"{user_id}|{password}|{user_name}"
        )

        action, content = parse_response(
            response
        )

        if action != "회원가입":
            return

        if content.startswith("OK|"):

            messagebox.showinfo(
                "회원가입",
                content.replace(
                    "OK|",
                    ""
                )
            )

            show_login_screen()

        else:

            messagebox.showerror(
                "회원가입 실패",
                content.replace(
                    "FAIL|",
                    ""
                )
            )


    signup_button = tk.Button(
        root,
        text="회원가입",
        width=20,
        font=("Arial", 13),
        command=signup
    )

    signup_button.pack(
        pady=20
    )


    back_button = tk.Button(
        root,
        text="뒤로",
        width=20,
        command=show_login_screen
    )

    back_button.pack()


# ============================================================
# 한글 검사
# ============================================================

def is_korean_only(text):

    for char in text:

        if char in " \t\n":
            continue

        if not (
            "가" <= char <= "힣"
        ):
            return False

    return True


# ============================================================
# 작업 화면
# ============================================================

def show_work_screen():

    clear_screen()

    create_date_header()

    title = tk.Label(
        root,
        text="번역 작업",
        font=("Arial", 22, "bold")
    )

    title.pack(pady=15)


    # 현재 사용자
    user_label = tk.Label(
        root,
        text=f"사용자 : {current_user}",
        font=("Arial", 12)
    )

    user_label.pack()


    # 폰트 크기
    font_frame = tk.Frame(root)

    font_frame.pack(
        pady=10
    )

    tk.Label(
        font_frame,
        text="폰트 크기 : "
    ).pack(side="left")


    font_size = tk.IntVar(
        value=10
    )


    # 언어 제한
    language_label = tk.Label(
        root,
        text="언어 제한 : 한글",
        font=("Arial", 11)
    )

    language_label.pack()


    # 크기에 따른 글자 제한
    limit_label = tk.Label(
        root,
        text="글자수 제한 : 10자",
        font=("Arial", 11)
    )

    limit_label.pack()


    # Text 위젯은 폰트 크기에 따라 실제 픽셀 크기가 변하므로
    # 고정 크기 Frame 안에 배치하여 입력/출력 영역의 크기를 일정하게 유지한다.
    input_frame = tk.Frame(root, width=520, height=150)
    input_frame.pack(padx=20, pady=10)
    input_frame.pack_propagate(False)

    input_text = tk.Text(
        input_frame,
        font=("Arial", 15),
        wrap="word"
    )

    input_text.pack(
        fill="both",
        expand=True
    )


    result_label = tk.Label(
        root,
        text="번역 결과",
        font=("Arial", 12, "bold")
    )

    result_label.pack(
        pady=(10, 3)
    )


    result_frame = tk.Frame(root, width=520, height=150)
    result_frame.pack(padx=20, pady=5)
    result_frame.pack_propagate(False)

    result_text = tk.Text(
        result_frame,
        font=("Arial", 15),
        state="disabled",
        wrap="word"
    )

    result_text.pack(
        fill="both",
        expand=True
    )


    # --------------------------------------------------------
    # 폰트 변경
    # --------------------------------------------------------

    def change_font():

        size = font_size.get()

        # 선택한 폰트 크기는 글자 수 제한에만 사용하고,
        # 입출력 영역의 글자 크기는 항상 15로 고정한다.
        input_text.config(
            font=("Arial", 15)
        )
        result_text.config(
            font=("Arial", 15)
        )

        if size == 5:
            limit = 15

        elif size == 10:
            limit = 10

        else:
            limit = 5

        limit_label.config(
            text=f"글자수 제한 : {limit}자"
        )


    for size in [5, 10, 15]:

        tk.Radiobutton(
            font_frame,
            text=str(size),
            variable=font_size,
            value=size,
            command=change_font
        ).pack(
            side="left",
            padx=5
        )


    # --------------------------------------------------------
    # 글자 입력 제한
    # --------------------------------------------------------

    def validate_input(event=None):

        size = font_size.get()

        if size == 5:
            max_length = 15

        elif size == 10:
            max_length = 10

        else:
            max_length = 5

        text = input_text.get(
            "1.0",
            "end-1c"
        )

        # 한글 검사
        if not is_korean_only(text):

            # 한글이 아닌 입력 제거.
            # 엔터(\n), 캐리지리턴(\r)은 입력값에서 무시한다.
            filtered = ""

            for char in text:

                if (
                    "가" <= char <= "힣"
                    or char in " \t"
                ):
                    filtered += char

            input_text.delete(
                "1.0",
                tk.END
            )

            input_text.insert(
                "1.0",
                filtered
            )

            text = filtered


        # 글자 수 제한
        if len(text) > max_length:

            input_text.delete(
                "1.0",
                tk.END
            )

            input_text.insert(
                "1.0",
                text[:max_length]
            )

        return "break"


    # KeyRelease 방식
    input_text.bind(
        "<KeyRelease>",
        validate_input
    )

    # 엔터 입력은 무시한다.
    input_text.bind(
        "<Return>",
        lambda event: "break"
    )


    # --------------------------------------------------------
    # 번역
    # --------------------------------------------------------

    def translate():

        text = input_text.get(
            "1.0",
            "end-1c"
        ).strip()

        if not text:

            messagebox.showwarning(
                "입력 오류",
                "번역할 한글을 입력하세요."
            )

            return

        response = send_request(
            "번역",
            f"{text}|{font_size.get()}"
        )

        action, content = parse_response(
            response
        )

        if action != "번역":
            return

        result_text.config(
            state="normal"
        )

        result_text.delete(
            "1.0",
            tk.END
        )

        if content.startswith("OK|"):

            result = content[3:]

            result_text.insert(
                "1.0",
                result
            )

        else:

            result_text.insert(
                "1.0",
                content.replace(
                    "FAIL|",
                    ""
                )
            )

        result_text.config(
            state="disabled"
        )


    translate_button = tk.Button(
        root,
        text="번역",
        width=20,
        font=("Arial", 13),
        command=translate
    )

    translate_button.pack(
        pady=10
    )


    # --------------------------------------------------------
    # 로그아웃
    # --------------------------------------------------------

    def logout():

        global logged_in
        global current_user

        if current_user:

            send_request(
                "로그아웃",
                current_user
            )

        logged_in = False
        current_user = None

        show_login_screen()


    logout_button = tk.Button(
        root,
        text="로그아웃",
        width=20,
        command=logout
    )

    logout_button.pack(
        pady=5
    )


# ============================================================
# 프로그램 종료
# ============================================================

def on_close():

    global sock

    if sock:

        try:

            if logged_in and current_user:

                send_request(
                    "로그아웃",
                    current_user
                )

            sock.close()

        except Exception:
            pass

    root.destroy()


# ============================================================
# 실행
# ============================================================

root = tk.Tk()

root.title(
    "Translation Client"
)

root.geometry(
    "600x700"
)

root.protocol(
    "WM_DELETE_WINDOW",
    on_close
)

show_connect_screen()

root.mainloop()
