#include "cron_analyse.h"

namespace Cron_Analyse
{
/*--------- 通用工具函数 ---------------------------------------------*/

/* 判断字符串是否纯数字 */
bool is_number(const char *s)
{
    if (!s || !*s) return 0;
    char *e = NULL;
    (void)strtol(s, &e, 10);
    return (*e == '\0');
}

/* 不区分大小写比较 */
int stricmp2(const char *a, const char *b)
{
    return strcasecmp(a, b);
}

/*--------- 月份/星期名称映射 ----------------------------------------*/
int month_name_to_int(const char *token)
{
    /* 支持 JAN, FEB, ..., DEC （大小写不敏感） */
    const char *names[] =
    {
        "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
        "JUL", "AUG", "SEP", "OCT", "NOV", "DEC",
        NULL
    };

    for(int i = 0; names[i]; ++i)
    {
        if (!stricmp2(token, names[i]))
        {
            return i + 1; /* 1~12 */
        }
    }

    return -1;
}
int dow_name_to_int(const char *token)
{
    /* 支持 SUN, MON, TUE, WED, THU, FRI, SAT */
    const char *names[] =
    {
        "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT",
        NULL
    };

    for(int i = 0; names[i]; ++i)
    {
        if (!stricmp2(token, names[i]))
        {
            return i; /* 0~6 */
        }
    }

    return -1;
}

/*--------- 日期辅助 -----------------------------------------------*/
/* 判断是否闰年 */
bool is_leap(int y)
{
    return ( (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0) );
}

/* 计算某年某月天数 */
int month_days(int y, int m)
{
    static const int t[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (m == 2)
    {
        return t[1] + is_leap(y);
    }

    return t[m - 1];
}

/* 返回当月 day号 附近的最近工作日（Mon-Fri） */
int nearest_weekday(int y, int m, int dom)
{
    int mdays = month_days(y, m);
    if (dom > mdays) dom = mdays;  /* 超过月底则取月底 */

    struct tm t;
    t.tm_year = y - 1900;
    t.tm_mon = m - 1;
    t.tm_mday = dom;
    t.tm_hour = 12;
    mktime(&t);

    /* 若本身是周一~周五，则直接 dom */
    if (t.tm_wday >= 1 && t.tm_wday <= 5)
    {
        return dom;
    }
    /* 若是周六 => 往前一天（周五）;但若是 1号 => 往后移到周一(3号) */
    if (t.tm_wday == 6)
    {
        if (dom == 1) return 3; /* 特例: 1号是周六 => 3号(周一) */
        return dom - 1;
    }
    /* 若是周日 => 往后一天（周一）;但若是月底 => -2 移到周五 */
    if (t.tm_wday == 0)
    {
        if (dom == mdays) return dom - 2; /* 末日周日 => 往前跳2天(周五) */
        return dom + 1;
    }
    return dom; /* fallback */
}

/* 计算本月最后一个 weekday(0~6) 所在日期 */
int last_weekday_in_month(int y, int m, int weekday)
{
    int mdays = month_days(y, m);
    /* 从月底往前找最多7天 */
    for(int d = mdays; d >= mdays - 6; --d)
    {
        struct tm t;
        t.tm_year = y - 1900;
        t.tm_mon = m - 1;
        t.tm_mday = d;
        t.tm_hour = 12;

        mktime(&t);
        if (t.tm_wday == weekday)
        {
            return d;
        }
    }
    return -1; /* 不会走到这里，除非 weekday 不在0~6 */
}

/* 计算本月第 nth(1~5) 个 weekday(0~6) 的日期 */
int nth_weekday(int y, int m, int weekday, int nth)
{
    struct tm t;
    t.tm_year = y - 1900;
    t.tm_mon = m - 1;
    t.tm_mday = 1;
    t.tm_hour = 12;
    mktime(&t);

    int offset = (weekday - t.tm_wday + 7) % 7;
    int d = 1 + offset + (nth - 1) * 7;
    if (d > month_days(y, m))
    {
        return -1; /* 不存在第 nth 个 weekday */
    }

    return d;
}

/*--------- 位图操作 (0-63) 用于秒/分/时/月/周快速匹配 ----------------*/
void bm_set(bitmap64 *bm, int v)
{
    *bm |= (1ULL << v);
}

int bm_test(const bitmap64 *bm, int v)
{
    return ((*bm >> v) & 1ULL);
}


/*=====================================================================
 *            解析部分
 *===================================================================*/

/* 给定字段类型，返回其取值范围 [lo, hi] */
void field_range(CFType t, int& lo, int& hi)
{
    switch(t)
    {
    case CF_SEC:
    case CF_MIN:
        lo = 0;
        hi = 59;
        break;
    case CF_HOUR:
        lo = 0;
        hi = 23;
        break;
    case CF_MON:
        lo = 1;
        hi = 12;
        break;
    case CF_DOW:
        lo = 0;
        hi = 7; /* 0或7 = 周日 */
        break;
    case CF_DOM:
        lo = 1;
        hi = 31;
        break;
    case CF_YEAR:
        lo = 1970;
        hi = 2099;
        break;
    }
}

/* 将可能含有英文缩写的 token 转为整数值 */
int parse_int_token(const char *tok, CFType t, int *v)
{
    if (t == CF_MON)
    {
        /* 尝试匹配 MONTH 名称 */
        int m = month_name_to_int(tok);
        if (m > 0)
        {
            *v = m;
            return 0;
        }
    }
    if (t == CF_DOW)
    {
        /* 尝试匹配 WEEKDAY 名称 */
        int d = dow_name_to_int(tok);
        if (d >= 0)
        {
            *v = d;
            return 0;
        }
    }
    /* 若是纯数字 */
    if (!is_number(tok))
    {
        return -1;
    }
    *v = atoi(tok);
    return 0;
}

/* 向 CronField 标记 val 可用 */
int field_mark_val(CronField *cf, int val)
{
    int lo, hi;
    field_range(cf->type, lo, hi);

    /* DOW 若写 7，当做 0 */
    if (cf->type == CF_DOW && val == 7)
    {
        val = 0;
    }
    if (val < lo || val > hi)
    {
        return -1;
    }

    /* 不同字段用不同结构记录 */
    if (cf->type == CF_SEC || cf->type == CF_MIN || cf->type == CF_HOUR ||
            cf->type == CF_MON || cf->type == CF_DOW)
    {
        bm_set(&cf->bits, val);
    }
    else if (cf->type == CF_DOM)
    {
        cf->day_list[val] = 1;
    }
    else if (cf->type == CF_YEAR)
    {
        cf->year_list[val - 1970] = 1;
    }
    return 0;
}

/* 解析一个形如 "1-5", "'*''/15", "3 - 9 / 2", "JAN", "MON", 等的范围片段 */
int parse_range(CronField *cf, const char *range)
{
    int lo, hi;
    field_range(cf->type, lo, hi);

    /* 是否有 '/' 步长 */
    int step = 1;
    char buf[32];
    const char *slash = strchr(range, '/');
    if (slash)
    {
        step = atoi(slash + 1);
        if (step <= 0)
        {
            return -1;
        }
        size_t len = slash - range;
        if (len >= sizeof(buf))
        {
            return -1;
        }
        strncpy(buf, range, len);
        buf[len] = '\0';
        range = buf; // 把range替换成 去掉 "/step"后的左侧那部分

        // 如果左侧那部分是 "*"，就代表 [lo - hi]
        if(!strcmp(range, "*"))
        {
            int start = lo;
            int end = hi;
            for (int val = start; val <= end; val += step)
            {
                if (field_mark_val(cf, val))
                {
                    return -1;
                }
            }
            return 0;
        }
    }

    /* 是否有 '-' 区间 */
    const char *dash = strchr(range, '-');
    int start, end;
    if (dash)
    {
        char left[16], right[16];
        size_t llen = dash - range;
        if (llen >= sizeof(left))
        {
            return -1;
        }
        strncpy(left, range, llen);
        left[llen] = '\0';
        strcpy(right, dash + 1);

        if (parse_int_token(left, cf->type, &start)) return -1;
        if (parse_int_token(right, cf->type, &end))   return -1;
        if (start > end)
        {
            int tmp = start;
            start = end;
            end = tmp;
        }
    }
    else
    {
        /* 单值 */
        if (parse_int_token(range, cf->type, &start))
        {
            return -1;
        }
        end = (slash ? hi : start);
    }

    /* 遍历并标记 */
    for(int val = start; val <= end; val += step)
    {
        if (field_mark_val(cf, val))
        {
            return -1;
        }
    }
    return 0;
}

/* 解析形如 "1,3-5,'*'/10" 的逗号列表 */
int parse_piece(CronField *cf, const char *piece)
{
    /*
     * 优先处理 DOM / DOW 特殊写法:
     *  1) L, LW, xW
     *  2) L-n, L-nW
     *  3) weekdayL  (e.g. 5L, FRIL)
     *  4) #n (MON#2, 3#1)
     *  5) C, xC
     */

    /*---------------- DOM 专属: L-n / L-nW ----------------*/
    if (cf->type == CF_DOM && !strncmp(piece, "L-", 2))
    {
        /* 例如 L-3, L-2W */
        const char *rest = piece + 2;
        char numbuf[8];
        size_t len = strspn(rest, "0123456789");
        if (len == 0 || len >= sizeof(numbuf))
        {
            return -1;
        }
        strncpy(numbuf, rest, len);
        numbuf[len] = '\0';
        int n = atoi(numbuf);
        if (n <= 0 || n > 31)
        {
            return -1;
        }
        cf->last_minus_n = n;
        rest += len;
        if (*rest == 'W')   /* L-nW */
        {
            cf->last_minus_w = 1;
            ++rest;
        }
        if (*rest != '\0')
        {
            return -1;  /* 不能再有别的字符 */
        }
        return 0;
    }

    /*----------------  DOM: L / LW / xW ----------------*/
    if (cf->type == CF_DOM)
    {
        if (!strcmp(piece, "L"))
        {
            cf->last_dom = 1;
            return 0;
        }
        if (!strcmp(piece, "LW"))
        {
            cf->lw = 1;
            return 0;
        }
        /* xW */
        size_t l = strlen(piece);
        if (l > 1 && piece[l - 1] == 'W')
        {
            char tmp[16];
            if (l - 1 >= sizeof(tmp)) return -1;
            strncpy(tmp, piece, l - 1);
            tmp[l - 1] = '\0';
            int x;
            if (parse_int_token(tmp, cf->type, &x)) return -1;
            cf->nearest_w = x;
            return 0;
        }
    }

    /*----------------  DOW: weekdayL (5L, FRIL) / #n  -----*/
    if (cf->type == CF_DOW)
    {
        size_t l = strlen(piece);
        /* weekdayL => 5L, MONL, FRIL */
        if (l > 1 && piece[l - 1] == 'L')
        {
            char tmp[16];
            if (l - 1 >= sizeof(tmp)) return -1;
            strncpy(tmp, piece, l - 1);
            tmp[l - 1] = '\0';

            int w;
            if (parse_int_token(tmp, cf->type, &w)) return -1;
            cf->last_wday     = 1;
            cf->last_wday_val = w;
            return 0;
        }
        /* # => 第n个周X   e.g. MON#2, 3#1 */
        char *hash = (char*)strchr(piece, '#');
        if (hash)
        {
            char left[16], right[16];
            size_t llen = hash - piece;
            if (llen >= sizeof(left)) return -1;
            strncpy(left, piece, llen);
            left[llen] = '\0';
            strcpy(right, hash + 1);

            int w;
            if (parse_int_token(left, cf->type, &w)) return -1;
            int n = atoi(right);
            if (n < 1 || n > 5) return -1;
            cf->nth_wday = w;
            cf->nth      = n;
            return 0;
        }
    }

    /*----------------  DOM / DOW: Calendar (C)  ---------*/
    if ((cf->type == CF_DOM || cf->type == CF_DOW))
    {
        char *cpos = (char*)strchr(piece, 'C');
        if (cpos && *(cpos + 1) == '\0') /* 末尾就是 C */
        {
            cf->use_calendar = 1;
            if (cpos == piece)
            {
                /* 纯 "C" */
                cf->cal_base = -1;
            }
            else
            {
                /* 形如 "5C" => base=5 */
                char basebuf[8];
                size_t blen = cpos - piece;
                if (blen >= sizeof(basebuf)) return -1;
                strncpy(basebuf, piece, blen);
                basebuf[blen] = '\0';
                if (parse_int_token(basebuf, cf->type, &cf->cal_base))
                {
                    return -1;
                }
            }
            return 0;
        }
    }

    /*----------------  通配符 * 与 ?  -------------------*/
    if (!strcmp(piece, "*"))
    {
        cf->any = 1;
        return 0;
    }
    if (!strcmp(piece, "?"))
    {
        if (cf->type != CF_DOM && cf->type != CF_DOW)
        {
            return -1; /* '?' 只允许在 DOM/DOW */
        }
        cf->ques = 1;
        return 0;
    }

    /*---------------- 正常解析 (range + step) ----------*/
    /* 可能是 "5" or "1-5" or "'*'/5" or "2 - 10 / 2" etc. */
    return parse_range(cf, piece);
}

/* 初始化 CronField */
void init_cron_field(CronField *cf, CFType t)
{
    cf->type     = t;
    cf->bits     = 0;
    memset(cf->day_list, 0, sizeof(cf->day_list));
    memset(cf->year_list, 0, sizeof(cf->year_list));

    cf->any  = 0;
    cf->ques = 0;

    cf->last_dom    = 0;
    cf->lw          = 0;
    cf->nearest_w   = 0;
    cf->last_wday   = 0;
    cf->last_wday_val = -1;
    cf->nth         = -1;
    cf->nth_wday    = -1;

    cf->last_minus_n   = 0;
    cf->last_minus_w   = 0;

    cf->use_calendar   = 0;
    cf->cal_base       = -1;
}

/* 解析单字段（可能带逗号） */
int parse_field(CronField *cf, const char *field)
{
    init_cron_field(cf, cf->type);

    char buf[128];
    if (strlen(field) >= sizeof(buf))
    {
        return -1;
    }
    strcpy(buf, field);

    int piece_cnt = 0;
    char *save = NULL;
    char *tok  = strtok_r(buf, ",", &save);
    while (tok)
    {
        if (parse_piece(cf, tok))
        {
            return -1;
        }
        tok = strtok_r(NULL, ",", &save);
        piece_cnt++;
    }
    /* 如果含 '*' 且却又有其它片段 => 错 */
    if (cf->any && piece_cnt > 1)
    {
        return -1;
    }
    return 0;
}

/* 解析表达式 => 填充 CronExpr */
int cron_parse(const char *expr, CronExpr *out, char *errmsg, size_t errsz)
{
    char copy[256];
    if (strlen(expr) >= sizeof(copy))
    {
        snprintf(errmsg, errsz, "expr too long");
        return -1;
    }
    strcpy(copy, expr);

    /* 分割出 6~7 字段 */
    char *fields[8] = {0};
    int n = 0;
    char *save = NULL, *tok = strtok_r(copy, " \t", &save);
    while (tok && n < 8)
    {
        fields[n++] = tok;
        tok = strtok_r(NULL, " \t", &save);
    }
    if (n < 6 || n > 7)
    {
        snprintf(errmsg, errsz, "need 6 or 7 fields");
        return -1;
    }

    out->has_year = (n == 7);

    /* 映射到 out->sec/min/hour/dom/mon/dow/(year) */
    struct
    {
        CronField* dst;
        CFType t;
    } map[] =
    {
        { &out->sec, CF_SEC },
        { &out->min, CF_MIN },
        { &out->hour, CF_HOUR},
        { &out->dom, CF_DOM },
        { &out->mon, CF_MON },
        { &out->dow, CF_DOW },
        { &out->year, CF_YEAR},
    };

    for (int i = 0; i < n; i++)
    {
        map[i].dst->type = map[i].t;
        if (parse_field(map[i].dst, fields[i]))
        {
            snprintf(errmsg, errsz, "field %d <%s> invalid", i + 1, fields[i]);
            return -1;
        }
    }
    if (!out->has_year)
    {
        /* 若无 year 字段 => 当作 '*' */
        out->year.any = 1;
    }

    /* 互斥检查：
       - 若 DOM 有 L-n 同时又 L / LW / xW / bits / ? => 冲突
       - Calendar(C) 也不能与其他(如 L / LW / xW...) 混用
       可根据自己需求做更严格校验。
     */
#define BADCOMBO_COUNT(cf) \
        ( (cf).any + (cf).ques + (cf).use_calendar + \
          (cf).last_dom + ((cf).last_minus_n > 0 ? 1 : 0) + (cf).lw + \
          ((cf).nearest_w!=0) + ((cf).bits!=0?1:0) + \
          ((cf).nth>0?1:0) + ((cf).last_wday>0?1:0) )

    /* 这里做简单判定：若一个字段多种模式叠加，就报错 */
    if (BADCOMBO_COUNT(out->dom) > 1)
    {
        snprintf(errmsg, errsz, "dom conflict usage");
        return -1;
    }
    if (BADCOMBO_COUNT(out->dow) > 1)
    {
        snprintf(errmsg, errsz, "dow conflict usage");
        return -1;
    }
#undef BADCOMBO_COUNT

    return 0;
}

/*=====================================================================
 *            匹配部分
 *===================================================================*/

/* 匹配字段常规值 */
int match_field(const CronField *cf, int y, int m, int dom, int dow, int val)
{
    if (cf->any)
    {
        return 1;
    }
    /* 对 秒/分/时/月/周/年 常规位图/数组 匹配 */
    if (cf->type == CF_SEC || cf->type == CF_MIN ||
            cf->type == CF_HOUR || cf->type == CF_MON || cf->type == CF_DOW)
    {
        if (cf->type == CF_DOW && val == 7)
        {
            val = 0;
        }
        return bm_test(&cf->bits, val);
    }
    if (cf->type == CF_YEAR)
    {
        if (val < 1970 || val > 2099) return 0;
        return cf->year_list[val - 1970];
    }

    /* DOM 特殊逻辑 */
    if (cf->type == CF_DOM)
    {
        int mdays = month_days(y, m);

        /* (1) L => 当月最后一天 */
        if (cf->last_dom)
        {
            return (dom == mdays);
        }

        /* (2) L-n / L-nW */
        if (cf->last_minus_n > 0)
        {
            int target = mdays - cf->last_minus_n;
            if (target < 1) target = 1; /* 边界保护 */

            if (cf->last_minus_w)
            {
                /* L-nW => 求 target 附近工作日 */
                target = nearest_weekday(y, m, target);
            }
            return (dom == target);
        }

        /* (3) LW => 当月最后工作日(最后一个周一~周五) */
        if (cf->lw)
        {
            /* 简化: 先算月末(一般31..28), 若是周六->-1, 若是周日->-2 */
            /* 这里也可调用 nearest_weekday(y,m,mdays) 再往前尝试 */
            int d = mdays;
            /* 往后 mktime 获取 wday */
            struct tm t;
            t.tm_year = y - 1900;
            t.tm_mon = m - 1;
            t.tm_mday = d;
            t.tm_hour = 12;
            mktime(&t);
            if (t.tm_wday == 6) d -= 1; /* 周六->周五 */
            else if (t.tm_wday == 0) d -= 2; /* 周日->周五 */
            return (dom == d);
        }

        /* (4) xW => 离 x号 最近的工作日 */
        if (cf->nearest_w)
        {
            int w = nearest_weekday(y, m, cf->nearest_w);
            return (dom == w);
        }

        /* (5) C => Calendar => e.g. "跳过周末"  */
        if (cf->use_calendar)
        {
            int base = (cf->cal_base == -1) ? 1 : cf->cal_base;
            if (base < 1) base = 1;
            if (base > mdays) base = mdays;
            int d = base;
            /* 找到 >= base 的第一个非周六日 */
            while (d <= mdays)
            {
                struct tm tmp;
                tmp.tm_year = y - 1900;
                tmp.tm_mon = m - 1;
                tmp.tm_mday = d;
                tmp.tm_hour = 12;
                mktime(&tmp);
                /* 仅把周六(6) 周日(0) 当作例外, 你也可对接自定义 isHoliday() */
                if (tmp.tm_wday != 0 && tmp.tm_wday != 6)
                {
                    break;
                }
                d++;
            }
            return (dom == d);
        }

        /* (6) 普通 day_list */
        if (dom >= 1 && dom <= 31)
        {
            return cf->day_list[dom];
        }
        return 0;
    }

    /* 若逻辑没覆盖到 => 不匹配 */
    return 0;
}

/* DOW 字段若也含 Calendar(C) => 同理处理 */
int match_field_dow_calendar(const CronField *cf, int dow)
{
    if (!cf->use_calendar)
    {
        /* 正常位图匹配 */
        if (cf->any) return 1;
        return bm_test(&cf->bits, (dow == 7 ? 0 : dow));
    }
    /* C =>  “baseC”，示例中仅演示跳过周末 */
    /* 对 base= xC:
       1) 先把 x 当做起点
       2) 如果 x==周末 => +1 直到非周末
     */
    int base = cf->cal_base;
    if (base < 0)
    {
        /* 纯 'C' => 视同: 以当前 dow 作为 base */
        base = dow;
    }
    int w = base % 7;
    while (w == 0 || w == 6)
    {
        w = (w + 1) % 7;
    }
    return (dow == w);
}

/* DOW 特殊模式: last_wday, nth_wday */
int match_dow_special(const CronField *cf, int y, int m, int dom, int dow)
{
    if (cf->last_wday)
    {
        /* “weekdayL” => 当月最后一个 weekdayVal */
        int lwday = last_weekday_in_month(y, m, cf->last_wday_val);
        return (dom == lwday);
    }
    if (cf->nth > 0 && cf->nth_wday >= 0)
    {
        /* "#n" => 第 n 个周X => 先算这天 */
        int d = nth_weekday(y, m, cf->nth_wday, cf->nth);
        return (dom == d);
    }
    return 0;
}

/* DOM & DOW 的综合判断(OR / AND / 互斥 / ? ) */
int match_dom_dow(const CronExpr *ce, const struct tm *tm)
{
    int y   = tm->tm_year + 1900;
    int m   = tm->tm_mon + 1;
    int dom = tm->tm_mday;
    int dow = tm->tm_wday; /* 0=周日..6=周六 */

    /* 先分别判断 DOM / DOW 是否匹配 */
    /* 1) DOM=? => 不关心dom
     * 2) DOW=? => 不关心dow
     * 3) L / L-n / #n / last_wday / calendar => match_field
     * 4) 另外 DOW 也可能 last_wday / nth_wday
     */
    int dom_q = ce->dom.ques;
    int dow_q = ce->dow.ques;

    /* DOW 里若是 last_wday 或 nth_wday, 需要看实际 dom */
    int dom_match = 0;
    if (!dom_q)
    {
        /* 走 DOM 常规匹配 */
        dom_match = match_field(&ce->dom, y, m, dom, dow, dom);
    }
    int dow_match = 0;
    if (!dow_q)
    {
        /* 先看 DOW 的 calendar(C) 或 bits */
        if (ce->dow.last_wday || ce->dow.nth > 0)
        {
            /* last_wday_in_month / nth_wday => 通过 dom来判断 */
            dow_match = match_dow_special(&ce->dow, y, m, dom, dow);
        }
        else if (ce->dow.use_calendar)
        {
            /* C => 仅判断 dow自己, dom不影响 */
            dow_match = match_field_dow_calendar(&ce->dow, dow);
        }
        else
        {
            /* 正常 DOW bitmap */
            if (ce->dow.any)
            {
                dow_match = 1;
            }
            else
            {
                dow_match = bm_test(&ce->dow.bits, (dow == 7 ? 0 : dow));
            }
        }
    }

    /* 若两个字段全是 '?' => 不限制 => 视为匹配 */
    if (dom_q && dow_q)
    {
        return 1;
    }

#if DOM_DOW_OR
    /* Quartz：如果两者都不是 '?'，就 OR */
    if (!dom_q && !dow_q)
    {
        return (dom_match || dow_match);
    }
    /* 若一方是 '?', 则只看另一方 */
    if (dom_q) return dow_match;
    if (dow_q) return dom_match;
    /* fallback (理应覆盖完) */
    return 0;
#else
    /* 传统 Linux: AND 逻辑 */
    if (dom_q) return dow_match;
    if (dow_q) return dom_match;
    return (dom_match && dow_match);
#endif
}

/* 总匹配入口 */
int cron_match(const CronExpr *ce, const struct tm *tm)
{
    int y   = tm->tm_year + 1900;
    int mo  = tm->tm_mon + 1; /* 月 */
    int dom = tm->tm_mday;    /* 日 */
    int dow = tm->tm_wday;    /* 周0~6 */
    int sec = tm->tm_sec;
    int min = tm->tm_min;
    int hr  = tm->tm_hour;

    /* 秒/分/时/月/年 => match_field 直接判断 */
    if (!match_field(&ce->sec, 0, 0, 0, 0, sec)) return 0;
    if (!match_field(&ce->min, 0, 0, 0, 0, min)) return 0;
    if (!match_field(&ce->hour, 0, 0, 0, 0, hr))  return 0;
    if (!match_field(&ce->mon, 0, 0, 0, 0, mo))  return 0;
    if (!match_field(&ce->year, 0, 0, 0, 0, y))    return 0;

    /* 最后判断 DOM + DOW */
    if (!match_dom_dow(ce, tm)) return 0;
    return 1;
}

}
