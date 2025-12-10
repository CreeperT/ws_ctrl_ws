#ifndef _CRON_ANALYSE_H_
#define _CRON_ANALYSE_H_

/************************************************************
 * Quartz-style CRON解析+匹配的实现
 *
 * 已覆盖：
 *   - 6~7字段（秒 分 时 日 月 周 [年]），支持 OR/AND 切换
 *   - 通配符: * ? - / , L LW W # C L-n L-nW
 *   - 月份/星期名称: JAN..DEC / SUN..SAT
 *   - 倒数第 n 日/工作日: L-n, L-nW
 *   - Calendar(C) 语义（简化为：跳过周末）
 *   - 可选年字段: 1970~2099
 *   - 健壮错误检测 & 线程安全
 ************************************************************/

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include <string>

/*---------------------------------------------------------------------*/
/* 可在此切换 日(DOM) 与 周(DOW) 同时为具体值时的逻辑
 *  1 => Quartz 的 OR 逻辑
 *  0 => 传统 Linux cron 的 AND 逻辑
 */
#define DOM_DOW_OR 1
/*---------------------------------------------------------------------*/

namespace Cron_Analyse
{
/*--------- 通用工具函数 ---------------------------------------------*/

/* 判断字符串是否纯数字 */
bool is_number(const char *s);

/* 不区分大小写比较 */
int stricmp2(const char *a, const char *b);


/*--------- 月份/星期名称映射 ----------------------------------------*/
int month_name_to_int(const char *token);

int dow_name_to_int(const char *token);


/*--------- 日期辅助 -----------------------------------------------*/
/* 判断是否闰年 */
bool is_leap(int y);

/* 计算某年某月天数 */
int month_days(int y, int m);

/* 返回当月 day号 附近的最近工作日（Mon-Fri） */
int nearest_weekday(int y, int m, int dom);

/* 计算本月最后一个 weekday(0~6) 所在日期 */
int last_weekday_in_month(int y, int m, int weekday);

/* 计算本月第 nth(1~5) 个 weekday(0~6) 的日期 */
int nth_weekday(int y, int m, int weekday, int nth);

/*--------- 位图操作 (0-63) 用于秒/分/时/月/周快速匹配 ----------------*/
typedef uint64_t bitmap64;

void bm_set(bitmap64 *bm, int v);

int bm_test(const bitmap64 *bm, int v);

/*--------- 字段类型 -------------------------------------------------*/
typedef enum
{
    CF_SEC,  /* 秒 */
    CF_MIN,  /* 分 */
    CF_HOUR, /* 时 */
    CF_DOM,  /* 日 */
    CF_MON,  /* 月 */
    CF_DOW,  /* 周 */
    CF_YEAR  /* 年(可选) */
} CFType;

/*--------- CronField -----------------------------------------------*/
/*
 * 这里使用:
 *   - bits  (位图存储 秒/分/时/月/周)
 *   - day_list[32], year_list[130] (存储 日/年)
 *   - 以及各种特殊标记
 */
typedef struct
{
    CFType type;
    bitmap64 bits;        /* 适用于秒、分、时、月、周 */

    /* 日 / 年 的可用标记 */
    uint8_t day_list[32];   /* 1-31 对应下标 */
    uint8_t year_list[130]; /* 1970-2099 => 下标 = year - 1970 */

    int any;      /* 是否 '*' */
    int ques;     /* 是否 '?' (仅 DOM / DOW) */

    /* L, LW, W, # 等之前已有的标记 */
    int last_dom;      /* 'L' (当月最后一天) */
    int lw;            /* 'LW' (当月最后工作日) */
    int nearest_w;     /* 'xW'  (离 x号最近的工作日) */
    int last_wday;     /* 'weekdayL' => 最后一个周X */
    int last_wday_val; /* 0~6 */
    int nth;           /* "#n" => 第 n 个周X (1~5) */
    int nth_wday;      /* 哪个周X (0~6) */

    /* 新增的 “倒数第 n 天(工作日)” L-n / L-nW */
    int last_minus_n;     /* n, 表示 "L-n" */
    int last_minus_w;     /* 是否带 W => "L-nW" */

    /* 新增的 Calendar(C) 语义 */
    int use_calendar; /* 是否含 'C' */
    int cal_base;     /* 若是 "xC" 则 base=x；若纯 "C" 则 base=-1 */
} CronField;

/*--------- CronExpr ------------------------------------------------*/
typedef struct
{
    CronField sec, min, hour, dom, mon, dow, year;
    bool has_year;
} CronExpr;

/*=====================================================================
 *            解析部分
 *===================================================================*/

/* 给定字段类型，返回其取值范围 [lo, hi] */
void field_range(CFType t, int& lo, int& hi);

/* 将可能含有英文缩写的 token 转为整数值 */
int parse_int_token(const char *tok, CFType t, int *v);

/* 向 CronField 标记 val 可用 */
int field_mark_val(CronField *cf, int val);

/* 解析一个形如 "1-5", "'*''/15", "3 - 9 / 2", "JAN", "MON", 等的范围片段 */
int parse_range(CronField *cf, const char *range);

/* 解析形如 "1,3-5,'*'/10" 的逗号列表 */
int parse_piece(CronField *cf, const char *piece);

/* 初始化 CronField */
void init_cron_field(CronField *cf, CFType t);

/* 解析单字段（可能带逗号） */
int parse_field(CronField *cf, const char *field);

/* 解析表达式 => 填充 CronExpr */
int cron_parse(const char *expr, CronExpr *out, char *errmsg, size_t errsz);

/*=====================================================================
 *            匹配部分
 *===================================================================*/

/* 匹配字段常规值 */
int match_field(const CronField *cf, int y, int m, int dom, int dow, int val);

/* DOW 字段若也含 Calendar(C) => 同理处理 */
int match_field_dow_calendar(const CronField *cf, int dow);

/* DOW 特殊模式: last_wday, nth_wday */
int match_dow_special(const CronField *cf, int y, int m, int dom, int dow);

/* DOM & DOW 的综合判断(OR / AND / 互斥 / ? ) */
int match_dom_dow(const CronExpr *ce, const struct tm *tm);

/* 总匹配入口 */
int cron_match(const CronExpr *ce, const struct tm *tm);

/*=====================================================================
 *   将 CronExpr 封装到 Task 中，并循环调度
 *===================================================================*/

typedef struct
{
    std::string expr_str; /* 原始CRON字符串 */
    CronExpr expr;        /* 解析结果 */
    bool parsed;           /* 是否解析成功 */
    char errmsg[64];      /* 若解析失败，存错误信息 */

    struct tm last;       /* 上次触发时间，避免同秒内连触发 */
} CronTask;

}

#endif
