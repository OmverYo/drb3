import os
import psycopg2
from datetime import date

conn = psycopg2.connect(
    host="localhost",
    port=5432,
    dbname="robotdb",
    user="rokey",
    password="rokey"
    )

cur = conn.cursor()

# 테이블 create
# 1. 사용자 정보 테이블
create_table_info_users = '''
CREATE TABLE info_users (
    user_id VARCHAR(10) NOT NULL,
    user_pw VARCHAR(10) NOT NULL,
    user_name VARCHAR(10) NOT NULL,
    registry_date DATE NOT NULL,
    recent_enter_date DATE NOT NULL,

    -- 기본키
    CONSTRAINT pk_info_users
        PRIMARY KEY (user_id),

    -- user_id:
    -- 영어 소문자(a-z)와 숫자(0-9)만 허용
    -- 1~10자, 띄어쓰기 불가
    CONSTRAINT chk_info_users_user_id
        CHECK (user_id ~ '^[a-z0-9]{1,10}$'),

    -- user_pw:
    -- 영어 소문자(a-z)와 숫자(0-9)만 허용
    -- 1~10자, 띄어쓰기 불가
    CONSTRAINT chk_info_users_user_pw
        CHECK (user_pw ~ '^[a-z0-9]{1,10}$'),

    -- user_name:
    -- 한글 또는 영어만 허용
    -- 1~10자
    CONSTRAINT chk_info_users_user_name
        CHECK (user_name ~ '^[가-힣A-Za-z]{1,10}$')
);
'''
# 2. 작업 내역 테이블
create_table_exec_translations = '''
CREATE TABLE exec_translations (
    exec_id BIGINT NOT NULL,
    user_id VARCHAR(10) NOT NULL,
    text VARCHAR(10) NOT NULL,
    request_date DATE NOT NULL,
    translate_result BOOLEAN NOT NULL DEFAULT FALSE,

    -- 기본키
    CONSTRAINT pk_exec_translations
        PRIMARY KEY (exec_id),

    -- 외래키
    -- exec_translations.user_id
    --     → info_users.user_id
    CONSTRAINT fk_exec_translations_user
        FOREIGN KEY (user_id)
        REFERENCES info_users(user_id),

    -- user_id:
    -- 영어 소문자(a-z)와 숫자(0-9)만 허용
    -- 1~10자, 띄어쓰기 불가
    CONSTRAINT chk_exec_translations_user_id
        CHECK (user_id ~ '^[a-z0-9]{1,10}$'),

    -- text:
    -- 한글만 허용
    -- 1~10자
    CONSTRAINT chk_exec_translations_text
        CHECK (text ~ '^[가-힣]{1,10}$'),

    -- TRUE / FALSE만 허용
    CONSTRAINT chk_exec_translations_result
        CHECK (translate_result IN (TRUE, FALSE))
);
'''

today = str(date.today())
# 데이터 insert
# 1. 사용자 회원가입 시
insert_data_info_users = f'''
INSERT INTO info_users
    (user_id, user_pw, user_name, registry_date, recent_enter_date)
VALUES
    ('rokey123', 'rokey123', '홍길동', '{today}', '{today}');
'''
# 2. 사용자 작업 수행 시
insert_data_exec_translations = f'''
INSERT INTO exec_translations
    (exec_id, user_id, text, request_date)
VALUES
    (1, 'rokey123', '안녕하세요', '{today}');
'''

today = str(date.today())
# 데이터 insert
# 1. 사용자 접속 시
update_data_info_users = f'''
UPDATE info_users
SET recent_enter_date = '{today}'
WHERE user_id = 'user01';
'''
# 2. 로봇 작업 결과 수신 시
update_data_exec_translations = f'''
UPDATE exec_translations
SET translate_result = TRUE
WHERE exec_id = 1;
'''

cur.execute(update_data_exec_translations)
conn.commit()

cur.close()
conn.close()



