/*
 * 中国交通路径规划系统
 * 修复版本 - 已解决所有审查报告中的问题
 *
 * 修复清单：
 *  [F1]  find_node() 中站点ID未加 city_count 偏移
 *  [F2]  添加 <limits.h> 以支持 INT_MAX
 *  [F3]  删除重复的图构建逻辑（原先构建了两次）
 *  [F4]  路径回溯添加边界检查，防止死循环和数组越界
 *  [F5]  find_city_id / find_station_id 改为排序+二分查找，O(log n)
 *  [F6]  Station 添加 has_coords 标志，不再以 (0,0) 作哨兵
 *  [F7]  find_node() 语义明确：飞机只查城市，火车只查车站
 *  [F8]  CSV 解析添加 \r\n 处理和空指针防御
 *  [F9]  MinHeap 容量改为 n*4，push() 满时动态扩容
 *  [F10] 添加 free_graph() 函数，程序结束前释放所有边内存
 *  [F11] 所有魔法数字提取为具名常量
 *  [F12] interactive_query() 添加输入范围验证
 *
 * 途径站支持（新增）：
 *  [T1]  引入 TrainSegRaw，统一存储所有车次的每段区间（含途径站）
 *  [T2]  替换 load_train_schedule_csv / load_train_slow_csv
 *        为 load_railway_csv，从 railway_line.csv 读取
 *        格式：车次,出发站,到达站,出发时间,到达时间（GB18030编码）
 *  [T3]  build_graph 中火车边直接使用真实时刻差，
 *        费用按站间哈弗辛距离 × 车次类型票价系数计算
 *  [T4]  删除 TrainSlowRaw 及相关代码（新CSV已统一覆盖所有车次）
 *  [W1]  Windows 控制台中文乱码：main() 开头调用 SetConsoleOutputCP(65001)
 *        切换为 UTF-8 代码页（仅在 _WIN32 下编译，Linux/Mac 无影响）
 *  [S1]  MAX_STATIONS 扩大为 8000（原3000不足以容纳 stations.csv 全部记录）
 *  [S2]  超限时错误提示更清晰，告知用户如何调整常量
 */

/*
 * [编码约定] 所有CSV数据文件统一使用UTF-8编码。
 * cities.csv / airports_cn.csv / china_flights.csv / stations.csv / stations2.csv
 * 若数据文件为GB18030/GBK编码，请先转换为UTF-8，否则中文站名比较会失败。
 * railway_line.csv 也需转为UTF-8，或确保编译/运行环境locale与文件编码一致。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>   /* [F2] INT_MAX */

/* Windows 控制台 UTF-8 支持 [W1] */
#ifdef _WIN32
#  include <windows.h>
#endif

/* ================================================================
   常量定义
   [F11] 所有魔法数字统一在此定义，禁止在代码中散落裸数字
   ================================================================ */
#define MAX_CITIES        600    /* 扩大：原500，余量充足 */
#define MAX_STATIONS      8000   /* 扩大：原3000不够，stations.csv可能超过3000行 */
#define MAX_FLIGHTS       10000
#define MAX_TRAIN_SEGS    50000   /* 途经站展开后约30000条区间 */
#define MAX_NAME_LEN      32
#define MAX_NODES         (MAX_CITIES + MAX_STATIONS)

#define PI                3.14159265358979323846
#define EARTH_RADIUS_KM   6371.0

/* 飞机参数 */
#define PLANE_SPEED_KMH       800.0   /* 巡航速度 km/h */
#define PLANE_BOARDING_MIN    30      /* 起降附加时间 分钟 */
#define PLANE_PRICE_PER_KM    0.8     /* 经济舱基准 元/km */
#define PLANE_SHORT_SURCHARGE 50      /* <500km 附加 */
#define PLANE_MID_SURCHARGE   80      /* 500~1000km 附加 */
#define PLANE_LONG_SURCHARGE  120     /* >1000km 附加 */
#define PLANE_SHORT_THRESHOLD 500.0
#define PLANE_MID_THRESHOLD   1000.0

/* 火车票价 元/km */
#define TRAIN_PRICE_HSR       0.50   /* 高铁 G */
#define TRAIN_PRICE_EMU       0.45   /* 动车 D/C */
#define TRAIN_PRICE_DIRECT    0.25   /* 直达 Z */
#define TRAIN_PRICE_EXPRESS   0.20   /* 特快 T */
#define TRAIN_PRICE_FAST      0.15   /* 快速 K */
#define TRAIN_PRICE_SLOW      0.10   /* 普快及以下 */

/* 火车速度 km/h（用于普快估算行驶时间） */
#define TRAIN_SPEED_G    280
#define TRAIN_SPEED_DC   180
#define TRAIN_SPEED_Z    110
#define TRAIN_SPEED_T    120
#define TRAIN_SPEED_K     90
#define TRAIN_SPEED_DEF   70

/* 停站附加时间：每100km加5分钟 */
#define TRAIN_STOP_MIN_PER_100KM  5

/* ================================================================
   枚举类型
   ================================================================ */
typedef enum { TRANSPORT_TRAIN, TRANSPORT_PLANE } TransportType;

/* ================================================================
   数据结构
   ================================================================ */

/* 城市节点 */
typedef struct {
    int    id;
    char   name[MAX_NAME_LEN];
    double lat, lon;
    char   iata[4];
} City;

/* 火车站节点
   [F6] 新增 has_coords 标志，不再用 (0,0) 判断坐标有效性 */
typedef struct {
    int    id;
    char   name[MAX_NAME_LEN];
    double lat, lon;
    int    city_id;
    int    has_coords;   /* [F6] 1=坐标有效, 0=无坐标 */
} Station;

/* 图的边（邻接表节点） */
typedef struct Edge {
    int           to_id;
    TransportType type;
    int           time_weight;     /* 纯行驶时间（分钟） */
    int           cost_weight;
    int           transfer_weight;
    int           depart_min;      /* 出发时刻(分钟)，-1表示无固定时刻 */
    char          schedule_name[20];
    struct Edge  *next;
} Edge;

/* 邻接表图 */
typedef struct {
    Edge    *adj[MAX_NODES];
    int      node_count;
    City     cities[MAX_CITIES];
    int      city_count;
    Station  stations[MAX_STATIONS];
    int      station_count;
} Graph;

/* [F5] 名称映射表（排序后用二分查找） */
typedef struct {
    char name[MAX_NAME_LEN];
    int  id;
} NameMap;

/* 原始数据缓冲区 */
typedef struct {
    char airline[8];
    char flight_no[10];
    char from_city[MAX_NAME_LEN];
    char to_city[MAX_NAME_LEN];
} FlightRaw;

/*
 * [T1] 统一的火车区间结构（含途径站）
 * 每行代表同一车次中某两站之间的一段区间，
 * 出发/到达时间为该区间的真实时刻（分钟数）。
 * 来源：railway_line.csv，格式 GB18030，
 * 列：车次,出发站,到达站,出发时间,到达时间
 */
typedef struct {
    char train_no[10];
    char from_station[MAX_NAME_LEN];
    char to_station[MAX_NAME_LEN];
    int  depart_min;   /* 出发时间，分钟数 0~1439（跨天则>1439） */
    int  arrive_min;   /* 到达时间，同上 */
} TrainSegRaw;

/* 最小堆节点 */
typedef struct {
    int node;
    int dist;
} HeapNode;

/* 最小堆 */
typedef struct {
    HeapNode *arr;
    int       size;
    int       capacity;
} MinHeap;

/* ================================================================
   全局变量
   ================================================================ */
static Graph g;
static int   g_json_mode = 0;   /* 1=JSON输出模式(供前端API), 0=交互模式 */

/* 日志宏：JSON模式下输出到stderr，避免污染stdout的JSON */
#define LOG(fmt, ...) do { \
    if (g_json_mode) fprintf(stderr, fmt "\n", ##__VA_ARGS__); \
    else printf(fmt "\n", ##__VA_ARGS__); \
} while(0)

static FlightRaw   flights[MAX_FLIGHTS];
static int         flight_cnt = 0;

/* [T1] 统一的火车区间数组，替代原先的 trains_sched + trains_slow */
static TrainSegRaw train_segs[MAX_TRAIN_SEGS];
static int         train_seg_cnt = 0;

/* 路径回溯辅助数组 */
static int           prev_node[MAX_NODES];
static int           prev_edge_time[MAX_NODES];
static int           prev_edge_cost[MAX_NODES];
static int           prev_wait[MAX_NODES];       /* 上车站的换乘等待时间（分钟） */
static char          prev_schedule[MAX_NODES][20];
static TransportType prev_type[MAX_NODES];

/* [F5] 名称映射表（排序后二分） */
static NameMap city_name_map[MAX_CITIES];
static int     city_map_cnt = 0;
static int     city_map_sorted = 0;   /* 标记是否已排序 */

static NameMap station_name_map[MAX_STATIONS];
static int     station_map_cnt = 0;
static int     station_map_sorted = 0;

/* ================================================================
   [F5] 二分查找辅助
   ================================================================ */
static int cmp_name_map(const void *a, const void *b) {
    return strcmp(((const NameMap *)a)->name,
                  ((const NameMap *)b)->name);
}

/* 对城市映射表排序（加载完成后调用一次） */
static void sort_city_map(void) {
    qsort(city_name_map, city_map_cnt, sizeof(NameMap), cmp_name_map);
    city_map_sorted = 1;
}

/* 对站点映射表排序 */
static void sort_station_map(void) {
    qsort(station_name_map, station_map_cnt, sizeof(NameMap), cmp_name_map);
    station_map_sorted = 1;
}

/* 查找城市id，不存在返回-1
   排序后 O(log n)，未排序退化为 O(n) */
static int find_city_id(const char *name) {
    if (city_map_sorted) {
        NameMap key;
        strncpy(key.name, name, MAX_NAME_LEN - 1);
        key.name[MAX_NAME_LEN - 1] = '\0';
        NameMap *res = bsearch(&key, city_name_map, city_map_cnt,
                               sizeof(NameMap), cmp_name_map);
        return res ? res->id : -1;
    }
    /* 排序前线性扫描 */
    for (int i = 0; i < city_map_cnt; i++) {
        if (strcmp(city_name_map[i].name, name) == 0)
            return city_name_map[i].id;
    }
    return -1;
}

/* 查找站点id，不存在返回-1 */
static int find_station_id(const char *name) {
    if (station_map_sorted) {
        NameMap key;
        strncpy(key.name, name, MAX_NAME_LEN - 1);
        key.name[MAX_NAME_LEN - 1] = '\0';
        NameMap *res = bsearch(&key, station_name_map, station_map_cnt,
                               sizeof(NameMap), cmp_name_map);
        return res ? res->id : -1;
    }
    for (int i = 0; i < station_map_cnt; i++) {
        if (strcmp(station_name_map[i].name, name) == 0)
            return station_name_map[i].id;
    }
    return -1;
}

/* ================================================================
   节点注册
   ================================================================ */
static int add_city(const char *name, double lat, double lon) {
    int id = find_city_id(name);
    if (id != -1) return id;
    if (g.city_count >= MAX_CITIES) {
        fprintf(stderr, "[WARN] City count reached limit %d / 城市数量已达上限 %d\n",
                MAX_CITIES, MAX_CITIES);
        return -1;
    }
    id = g.city_count++;
    strncpy(g.cities[id].name, name, MAX_NAME_LEN - 1);
    g.cities[id].name[MAX_NAME_LEN - 1] = '\0';
    g.cities[id].lat  = lat;
    g.cities[id].lon  = lon;
    g.cities[id].iata[0] = '\0';
    g.cities[id].id   = id;

    strncpy(city_name_map[city_map_cnt].name, name, MAX_NAME_LEN - 1);
    city_name_map[city_map_cnt].name[MAX_NAME_LEN - 1] = '\0';
    city_name_map[city_map_cnt].id = id;
    city_map_cnt++;
    city_map_sorted = 0;   /* 新增后需重新排序 */
    return id;
}

/* [F6] has_coords 参数控制坐标有效性 */
static int add_station(const char *name, double lat, double lon,
                       int city_id, int has_coords) {
    int id = find_station_id(name);
    if (id != -1) return id;
    if (g.station_count >= MAX_STATIONS) {
        fprintf(stderr,
            "[WARN] Station count reached limit %d / 站点数量已达上限 %d\n"
            "       Increase MAX_STATIONS in source and recompile.\n"
            "       请将源码中 MAX_STATIONS 调大后重新编译。\n",
            MAX_STATIONS, MAX_STATIONS);
        return -1;
    }
    id = g.station_count++;
    strncpy(g.stations[id].name, name, MAX_NAME_LEN - 1);
    g.stations[id].name[MAX_NAME_LEN - 1] = '\0';
    g.stations[id].lat        = lat;
    g.stations[id].lon        = lon;
    g.stations[id].city_id    = city_id;
    g.stations[id].has_coords = has_coords;   /* [F6] */
    g.stations[id].id         = id;

    strncpy(station_name_map[station_map_cnt].name, name, MAX_NAME_LEN - 1);
    station_name_map[station_map_cnt].name[MAX_NAME_LEN - 1] = '\0';
    station_name_map[station_map_cnt].id = id;
    station_map_cnt++;
    station_map_sorted = 0;
    return id;
}

/* ================================================================
   地理与票价计算
   ================================================================ */
static double haversine(double lat1, double lon1,
                         double lat2, double lon2) {
    double dlat = (lat2 - lat1) * PI / 180.0;
    double dlon = (lon2 - lon1) * PI / 180.0;
    double a = sin(dlat/2) * sin(dlat/2)
             + cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0)
             * sin(dlon/2) * sin(dlon/2);
    return EARTH_RADIUS_KM * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

static int calc_plane_price(double distance) {
    int base = (int)(distance * PLANE_PRICE_PER_KM);
    if      (distance < PLANE_SHORT_THRESHOLD) base += PLANE_SHORT_SURCHARGE;
    else if (distance < PLANE_MID_THRESHOLD)   base += PLANE_MID_SURCHARGE;
    else                                        base += PLANE_LONG_SURCHARGE;
    return base;
}

static int calc_plane_time(double distance) {
    return (int)(distance / PLANE_SPEED_KMH * 60.0) + PLANE_BOARDING_MIN;
}

/* 根据车次前缀返回速度 km/h */
static int get_train_speed(const char *train_no) {
    if (!train_no || train_no[0] == '\0') return TRAIN_SPEED_DEF;
    switch (train_no[0]) {
        case 'G':                    return TRAIN_SPEED_G;
        case 'D': case 'C':          return TRAIN_SPEED_DC;
        case 'Z':                    return TRAIN_SPEED_Z;
        case 'T':                    return TRAIN_SPEED_T;
        case 'K':                    return TRAIN_SPEED_K;
        default:                     return TRAIN_SPEED_DEF;
    }
}

/* 根据车次前缀返回票价系数 元/km */
static double get_train_price_rate(const char *train_no) {
    if (!train_no || train_no[0] == '\0') return TRAIN_PRICE_SLOW;
    switch (train_no[0]) {
        case 'G':           return TRAIN_PRICE_HSR;
        case 'D': case 'C': return TRAIN_PRICE_EMU;
        case 'Z':           return TRAIN_PRICE_DIRECT;
        case 'T':           return TRAIN_PRICE_EXPRESS;
        case 'K':           return TRAIN_PRICE_FAST;
        default:            return TRAIN_PRICE_SLOW;
    }
}

/* ================================================================
   [F8] CSV 解析工具：去除行尾 \r\n
   ================================================================ */
static void strip_line(char *line) {
    size_t len = strlen(line);
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
        line[--len] = '\0';
}

/* ================================================================
   数据加载
   ================================================================ */
static void load_cities_csv(void) {
    FILE *fp = fopen("cities.csv", "rb");
    if (!fp) { fprintf(stderr, "[ERROR] Cannot open cities.csv / 无法打开 cities.csv\n"); return; }
    char line[256];
    fgets(line, sizeof(line), fp); /* 跳过标题行 */
    while (fgets(line, sizeof(line), fp)) {
        strip_line(line);                           /* [F8] */
        char *name  = strtok(line, ",");
        char *s_lat = strtok(NULL, ",");
        char *s_lon = strtok(NULL, ",");
        if (!name || !s_lat || !s_lon) continue;    /* [F8] 空指针防御 */
        add_city(name, atof(s_lat), atof(s_lon));
    }
    fclose(fp);
    sort_city_map();   /* [F5] 加载完成后排序 */
    LOG("已加载 %d 个城市", g.city_count);
}

static void load_airports_csv(void) {
    FILE *fp = fopen("airports_cn.csv", "rb");
    if (!fp) { fprintf(stderr, "[ERROR] Cannot open airports_cn.csv / 无法打开 airports_cn.csv\n"); return; }
    char line[512];
    fgets(line, sizeof(line), fp);
    while (fgets(line, sizeof(line), fp)) {
        strip_line(line);                           /* [F8] */
        char *iata    = strtok(line, ",");
        if (!iata) continue;
        strtok(NULL, ",");                          /* skip city_en */
        char *city_cn = strtok(NULL, ",");
        if (!city_cn) continue;
        int cid = find_city_id(city_cn);
        if (cid >= 0) {
            strncpy(g.cities[cid].iata, iata, 3);
            g.cities[cid].iata[3] = '\0';
        }
    }
    fclose(fp);
    LOG("机场信息已匹配到城市");
}

static void load_flights_csv(void) {
    FILE *fp = fopen("china_flights.csv", "rb");
    if (!fp) { fprintf(stderr, "[ERROR] Cannot open china_flights.csv / 无法打开 china_flights.csv\n"); return; }
    char line[256];
    fgets(line, sizeof(line), fp);
    while (fgets(line, sizeof(line), fp)) {
        if (flight_cnt >= MAX_FLIGHTS) break;
        strip_line(line);                           /* [F8] */
        char *airline    = strtok(line, ",");
        char *flight_no  = strtok(NULL, ",");
        if (!airline || !flight_no) continue;
        strtok(NULL, ",");                          /* skip from_iata */
        char *from_city  = strtok(NULL, ",");
        if (!from_city) continue;
        strtok(NULL, ",");                          /* skip to_iata */
        char *to_city    = strtok(NULL, ",");
        if (!to_city) continue;

        if (find_city_id(from_city) < 0 || find_city_id(to_city) < 0)
            continue;

        strncpy(flights[flight_cnt].airline,    airline,   7);
        strncpy(flights[flight_cnt].flight_no,  flight_no, 9);
        strncpy(flights[flight_cnt].from_city,  from_city, MAX_NAME_LEN - 1);
        strncpy(flights[flight_cnt].to_city,    to_city,   MAX_NAME_LEN - 1);
        flight_cnt++;
    }
    fclose(fp);
    LOG("已加载 %d 条航班记录", flight_cnt);
}

static void load_stations_csv(void) {
    /* 加载 stations.csv */
    FILE *fp = fopen("stations.csv", "rb");
    if (fp) {
        char line[512];
        fgets(line, sizeof(line), fp);
        while (fgets(line, sizeof(line), fp)) {
            strip_line(line);                       /* [F8] */
            char *name  = strtok(line, ",");
            strtok(NULL, ",");                      /* 跳过电报码列(如 BJP, VNP) */
            char *s_lon = strtok(NULL, ",");
            char *s_lat = strtok(NULL, ",");
            if (!name || !s_lon || !s_lat) continue;
            double lon = atof(s_lon), lat = atof(s_lat);
            int city_id = -1;
            /* 前缀匹配：站名通常以城市名开头，选最长匹配 */
            int best_len = 0;
            for (int i = 0; i < g.city_count; i++) {
                size_t clen = strlen(g.cities[i].name);
                if (clen > (size_t)best_len &&
                    strncmp(name, g.cities[i].name, clen) == 0) {
                    best_len = (int)clen;
                    city_id = i;
                }
            }
            add_station(name, lat, lon, city_id, 1);   /* [F6] has_coords=1 */
        }
        fclose(fp);
    }

    /* 加载 stations2.csv（补充） */
    fp = fopen("stations2.csv", "rb");
    if (fp) {
        char line[512];
        fgets(line, sizeof(line), fp);
        while (fgets(line, sizeof(line), fp)) {
            strip_line(line);
            char *name  = strtok(line, ",");
            char *s_lon = strtok(NULL, ",");
            char *s_lat = strtok(NULL, ",");
            if (!name || !s_lon || !s_lat) continue;
            if (find_station_id(name) != -1) continue;
            double lon = atof(s_lon), lat = atof(s_lat);
            int city_id = -1;
            int best_len = 0;
            for (int i = 0; i < g.city_count; i++) {
                size_t clen = strlen(g.cities[i].name);
                if (clen > (size_t)best_len &&
                    strncmp(name, g.cities[i].name, clen) == 0) {
                    best_len = (int)clen;
                    city_id = i;
                }
            }
            add_station(name, lat, lon, city_id, 1);
        }
        fclose(fp);
    }

    sort_station_map();   /* [F5] 加载完成后排序 */
    LOG("已加载 %d 个站点", g.station_count);
}

/*
 * [T2] load_railway_csv
 * 读取 railway_line.csv（UTF-8编码），格式：
 *   车次,途经站列表,始发站,终点站,出发时间,到达时间
 * 途经站以"、"分隔，不含ASCII逗号；第2列整体被跳过。
 * 时间格式为 "H:MM:SS" 或 "HH:MM:SS"，只取小时和分钟。
 * 每行代表一段区间（含途径站），同一车次可有多行。
 * 若出发/到达站在 stations.csv 中未找到（无坐标），
 * 仍注册站点节点（has_coords=0），后续构建边时按距离过滤。
 */
static void load_railway_csv(void) {
    /*
     * 直接以二进制模式打开 GB18030 文件。
     * 逗号、数字、冒号均为单字节 ASCII，strtok 切分完全正确。
     * 中文站名作为不透明字节串存储，strcmp 在同编码环境下比较正确。
     * 无需 iconv，跨平台兼容。
     *
     * 自动尝试多个候选文件名，无需用户手动重命名。
     */
    static const char *candidates[] = {
        "railway_line.csv",
        "railway_line_1_.csv",
        "railway_line_3_.csv",
        "railway_line_1.csv",
        "railway_line_3.csv",
        NULL
    };
    FILE *fp = NULL;
    const char *opened = NULL;
    for (int i = 0; candidates[i]; i++) {
        fp = fopen(candidates[i], "rb");
        if (fp) { opened = candidates[i]; break; }
    }
    if (!fp) {
        fprintf(stderr,
            "[ERROR] Railway data file not found. / 找不到铁路数据文件。\n"
            "        Tried: railway_line.csv, railway_line_1_.csv, railway_line_3_.csv\n"
            "        Please place one of these files next to the executable.\n");
        return;
    }
    LOG("已找到铁路数据文件: %s", opened);

    char line[1024];
    fgets(line, sizeof(line), fp);  /* 跳过标题行 */

    while (fgets(line, sizeof(line), fp)) {
        if (train_seg_cnt >= MAX_TRAIN_SEGS) {
            fprintf(stderr,
                "[WARN] Train segment limit %d reached, rest ignored. / "
                "火车区间数超过上限 %d，后续记录已忽略\n",
                MAX_TRAIN_SEGS, MAX_TRAIN_SEGS);
            break;
        }
        strip_line(line);                           /* [F8] 去除 \r\n */

        char *train_no  = strtok(line, ",");
        char *waypoints = strtok(NULL, ",");          /* 途经站列表（"、"分隔） */
        char *from      = strtok(NULL, ",");
        char *to        = strtok(NULL, ",");
        char *depart    = strtok(NULL, ",");
        char *arrive    = strtok(NULL, ",");
        if (!train_no || !from || !to || !depart || !arrive) continue;

        /* 解析 "H:MM:SS" 或 "HH:MM:SS"，只取时:分 */
        int dh = 0, dm = 0, ds = 0;
        int ah = 0, am = 0, as_ = 0;
        if (sscanf(depart, "%d:%d:%d", &dh, &dm, &ds) < 2) continue;
        if (sscanf(arrive, "%d:%d:%d", &ah, &am, &as_) < 2) continue;

        int depart_min = dh * 60 + dm;
        int arrive_min = ah * 60 + am;
        /* 跨天：到达时间早于出发时间，循环加24小时（支持>24h行程） */
        while (arrive_min < depart_min) arrive_min += 24 * 60;
        int total_time = arrive_min - depart_min;
        if (total_time <= 0) continue;

        /* 拆分途经站列表（以UTF-8"、"分隔） */
        #define MAX_WP 128
        char *wp_names[MAX_WP];
        int wp_cnt = 0;
        char *rest = (waypoints && waypoints[0]) ? waypoints : NULL;
        if (rest) {
            while (*rest && wp_cnt < MAX_WP) {
                char *next = strstr(rest, "、");
                if (next) {
                    *next = '\0';
                    wp_names[wp_cnt++] = rest;
                    rest = next + strlen("、");
                } else {
                    wp_names[wp_cnt++] = rest;
                    break;
                }
            }
        }
        /* 途经站不足2个，回退为整段 from→to */
        if (wp_cnt < 2) {
            wp_names[0] = from;
            wp_names[1] = to;
            wp_cnt = 2;
        }

        /* 注册所有途经站（若已存在则直接返回已有id） */
        for (int j = 0; j < wp_cnt; j++) {
            if (find_station_id(wp_names[j]) == -1)
                add_station(wp_names[j], 0.0, 0.0, -1, 0);
        }

        /* 按哈弗辛距离比例分配每段时间 */
        double seg_dists[MAX_WP];
        double total_dist = 0.0;
        int all_coords = 1;
        for (int j = 0; j < wp_cnt - 1; j++) {
            int s1 = find_station_id(wp_names[j]);
            int s2 = find_station_id(wp_names[j+1]);
            if (s1 >= 0 && s2 >= 0 &&
                g.stations[s1].has_coords && g.stations[s2].has_coords) {
                seg_dists[j] = haversine(g.stations[s1].lat, g.stations[s1].lon,
                                         g.stations[s2].lat, g.stations[s2].lon);
            } else {
                seg_dists[j] = 0.0;
                all_coords = 0;
            }
            total_dist += seg_dists[j];
        }
        if (total_dist < 0.1) all_coords = 0;

        /* 为每段相邻站点创建区间记录 */
        int cum_time = depart_min;
        for (int j = 0; j < wp_cnt - 1; j++) {
            if (train_seg_cnt >= MAX_TRAIN_SEGS) break;
            int seg_time;
            if (all_coords) {
                seg_time = (int)(total_time * seg_dists[j] / total_dist);
            } else {
                seg_time = total_time / (wp_cnt - 1);
            }
            if (seg_time < 1) seg_time = 1;
            int seg_arrive = cum_time + seg_time;

            strncpy(train_segs[train_seg_cnt].train_no,     train_no, 9);
            train_segs[train_seg_cnt].train_no[9] = '\0';
            strncpy(train_segs[train_seg_cnt].from_station, wp_names[j], MAX_NAME_LEN - 1);
            train_segs[train_seg_cnt].from_station[MAX_NAME_LEN - 1] = '\0';
            strncpy(train_segs[train_seg_cnt].to_station,   wp_names[j+1], MAX_NAME_LEN - 1);
            train_segs[train_seg_cnt].to_station[MAX_NAME_LEN - 1] = '\0';
            train_segs[train_seg_cnt].depart_min = cum_time;
            train_segs[train_seg_cnt].arrive_min = seg_arrive;
            train_seg_cnt++;
            cum_time = seg_arrive;
        }
    }
    fclose(fp);

    /* 新站点已加入，重新排序映射表 [F5] */
    sort_station_map();

    LOG("已加载 %d 条火车区间记录（含途径站）", train_seg_cnt);
}

/* ================================================================
   图的构建与释放
   ================================================================ */
static void add_edge(int from_id, int to_id, TransportType type,
                     int time, int cost, const char *sched_name,
                     int depart_min) {
    Edge *e = (Edge *)malloc(sizeof(Edge));
    if (!e) { fprintf(stderr, "[ERROR] Out of memory in add_edge / 内存不足：add_edge\n"); return; }
    e->to_id           = to_id;
    e->type            = type;
    e->time_weight     = time;
    e->cost_weight     = cost;
    e->transfer_weight = 1;
    e->depart_min      = depart_min;
    strncpy(e->schedule_name, sched_name, 19);
    e->schedule_name[19] = '\0';
    e->next            = g.adj[from_id];
    g.adj[from_id]     = e;
}

/* [F10] 释放图中所有边 */
static void free_graph(void) {
    int n = g.city_count + g.station_count;
    for (int i = 0; i < n; i++) {
        Edge *e = g.adj[i];
        while (e) {
            Edge *nx = e->next;
            free(e);
            e = nx;
        }
        g.adj[i] = NULL;
    }
}

/* [F3] 只构建一次图，使用正确的全局节点编号
 *
 * 节点编号：
 *   0 .. city_count-1                       : 城市节点（仅飞机边使用）
 *   city_count .. city_count+station_count-1 : 车站节点（火车边使用）
 *
 * 不使用城市虚拟节点+walk边：walk边权重为0会干扰Dijkstra权重计算，
 * 导致绕路。改为多起点/多终点Dijkstra处理用户输入城市名的情况。
 */
static void build_graph(void) {
    int city_offset = g.city_count;
    int n = city_offset + g.station_count;

    for (int i = 0; i < n; i++) g.adj[i] = NULL;

    /* ---- 飞机边：城市节点 ---- */
    for (int i = 0; i < flight_cnt; i++) {
        int from = find_city_id(flights[i].from_city);
        int to   = find_city_id(flights[i].to_city);
        if (from < 0 || to < 0) continue;
        double dist = haversine(g.cities[from].lat, g.cities[from].lon,
                                g.cities[to].lat,   g.cities[to].lon);
        add_edge(from, to, TRANSPORT_PLANE,
                 calc_plane_time(dist), calc_plane_price(dist),
                 flights[i].flight_no, -1);   /* 飞机无固定时刻 */
    }

    /* ---- 火车边：车站节点 ---- */
    int train_edge_cnt = 0;
    for (int i = 0; i < train_seg_cnt; i++) {
        int fs = find_station_id(train_segs[i].from_station);
        int ts = find_station_id(train_segs[i].to_station);
        if (fs < 0 || ts < 0) continue;

        int travel = train_segs[i].arrive_min - train_segs[i].depart_min;
        if (travel <= 0) continue;

        int cost;
        if (g.stations[fs].has_coords && g.stations[ts].has_coords) {
            double dist = haversine(g.stations[fs].lat, g.stations[fs].lon,
                                    g.stations[ts].lat, g.stations[ts].lon);
            if (dist < 0.1) continue;
            cost = (int)(dist * get_train_price_rate(train_segs[i].train_no));
        } else {
            int speed = get_train_speed(train_segs[i].train_no);
            double est_dist = (double)travel / 60.0 * speed;
            cost = (int)(est_dist * get_train_price_rate(train_segs[i].train_no));
        }
        if (cost < 5) cost = 5;

        add_edge(city_offset + fs, city_offset + ts,
                 TRANSPORT_TRAIN, travel, cost, train_segs[i].train_no,
                 train_segs[i].depart_min);
        train_edge_cnt++;
    }

    LOG("图构建完成：%d个节点（%d城市+%d站点），%d条火车边",
           n, g.city_count, g.station_count, train_edge_cnt);
}

/* ================================================================
   最小堆（[F9] 动态扩容）
   ================================================================ */
static MinHeap *create_heap(int capacity) {
    MinHeap *h = (MinHeap *)malloc(sizeof(MinHeap));
    if (!h) return NULL;
    h->arr = (HeapNode *)malloc(sizeof(HeapNode) * capacity);
    if (!h->arr) { free(h); return NULL; }
    h->size     = 0;
    h->capacity = capacity;
    return h;
}

static void free_heap(MinHeap *h) {
    if (h) { free(h->arr); free(h); }
}

static void heap_swap(HeapNode *a, HeapNode *b) {
    HeapNode t = *a; *a = *b; *b = t;
}

/* [F9] 堆满时自动扩容（容量翻倍） */
static int heap_push(MinHeap *h, int node, int dist) {
    if (h->size == h->capacity) {
        int new_cap = h->capacity * 2;
        HeapNode *tmp = (HeapNode *)realloc(h->arr,
                                            sizeof(HeapNode) * new_cap);
        if (!tmp) {
            fprintf(stderr, "[ERROR] Heap realloc failed / 堆扩容失败\n");
            return 0;
        }
        h->arr      = tmp;
        h->capacity = new_cap;
    }
    int i = h->size++;
    h->arr[i].node = node;
    h->arr[i].dist = dist;
    while (i > 0 && h->arr[(i-1)/2].dist > h->arr[i].dist) {
        heap_swap(&h->arr[(i-1)/2], &h->arr[i]);
        i = (i-1)/2;
    }
    return 1;
}

static HeapNode heap_pop(MinHeap *h) {
    HeapNode min = h->arr[0];
    h->arr[0] = h->arr[--h->size];
    int i = 0;
    for (;;) {
        int l = 2*i+1, r = 2*i+2, s = i;
        if (l < h->size && h->arr[l].dist < h->arr[s].dist) s = l;
        if (r < h->size && h->arr[r].dist < h->arr[s].dist) s = r;
        if (s == i) break;
        heap_swap(&h->arr[i], &h->arr[s]);
        i = s;
    }
    return min;
}

/* ================================================================
 * find_nodes：将用户输入的名称解析为候选全局节点ID列表
 *
 * 火车模式：
 *   - 精确匹配站名 → 只返回该站
 *   - 匹配城市名   → 返回该城市下所有站（站名以城市名开头）
 *   这样用户输入「北京」可同时覆盖「北京」「北京南」「北京西」
 *
 * 飞机模式：只返回城市节点（单个）
 *
 * 返回值：候选节点数（0表示未找到）
 * out_ids[]：调用方提供的缓冲区，容量至少MAX_STATIONS
 * ================================================================ */
#define MAX_CANDIDATES 64   /* 同城站点上限，实际不超过10个 */

static int find_nodes(const char *name, TransportType type,
                      int out_ids[], int max_out) {
    int cnt = 0;

    if (type == TRANSPORT_PLANE) {
        int cid = find_city_id(name);
        if (cid >= 0 && cnt < max_out) out_ids[cnt++] = cid;
        return cnt;
    }

    /* 火车：先尝试精确匹配站名 */
    int sid = find_station_id(name);
    int cid = find_city_id(name);

    /* 精确匹配到站名，且该名称不是城市名 → 直接返回该站 */
    if (sid >= 0 && cid < 0) {
        out_ids[cnt++] = g.city_count + sid;
        return cnt;
    }

    /* 否则：按城市名/站名前缀匹配，收集所有车站
     * 匹配规则：站名以城市名开头（字节级别前缀匹配）
     * 所有CSV文件编码需一致（均为UTF-8）才能正确比较 */
    if (cid < 0 && sid < 0) {
        fprintf(stderr, "[WARN] Not found: %s\n", name);
        return 0;
    }
    const char *prefix = (cid >= 0) ? name : g.stations[sid].name;
    size_t clen = strlen(prefix);
    for (int i = 0; i < g.station_count && cnt < max_out; i++) {
        if (strncmp(g.stations[i].name, prefix, clen) == 0) {
            out_ids[cnt++] = g.city_count + i;
        }
    }
    if (cnt == 0)
        fprintf(stderr, "[WARN] No matching stations found for '%s'\n", name);
    return cnt;
}

/* ================================================================
 * print_path：回溯并打印路径，跳过权重为0的walk步骤
 * ================================================================ */
static void print_path(int end) {
    int path[MAX_NODES], path_len = 0;
    int v = end;
    while (v != -1 && path_len < MAX_NODES) {
        path[path_len++] = v;
        v = prev_node[v];
    }
    if (path_len >= MAX_NODES)
        fprintf(stderr, "警告：路径过长，可能存在环\n");

    /* 辅助：节点ID → 名称 */
    #define NODE_NAME(buf, id) do { \
        if ((id) < g.city_count) { \
            strncpy(buf, g.cities[id].name, MAX_NAME_LEN - 1); \
            buf[MAX_NAME_LEN - 1] = '\0'; \
        } else { \
            strncpy(buf, g.stations[(id) - g.city_count].name, MAX_NAME_LEN - 1); \
            buf[MAX_NAME_LEN - 1] = '\0'; \
        } \
    } while(0)

    printf("详细行程：\n");

    int  merge_from  = -1;          /* 合并段的起始节点 */
    int  merge_time  = 0;
    int  merge_cost  = 0;
    char merge_sched[20] = "";
    TransportType merge_type = TRANSPORT_TRAIN;

    for (int i = path_len - 1; i > 0; i--) {
        int u = path[i], w = path[i-1];
        if (prev_edge_time[w] == 0 && prev_edge_cost[w] == 0) continue;

        int same = (merge_from >= 0 &&
                    strcmp(prev_schedule[w], merge_sched) == 0 &&
                    prev_type[w] == merge_type);

        if (same) {
            merge_time += prev_edge_time[w];
            merge_cost += prev_edge_cost[w];
        } else {
            /* 输出上一段合并结果 */
            if (merge_from >= 0) {
                char fn[MAX_NAME_LEN], tn[MAX_NAME_LEN];
                NODE_NAME(fn, merge_from);
                NODE_NAME(tn, u);
                const char *ts = (merge_type == TRANSPORT_PLANE) ? "飞机" : "火车";
                printf("  乘坐%s[%s]  从[%s]到[%s]  耗时%d分钟，票价%d元\n",
                       ts, merge_sched, fn, tn, merge_time, merge_cost);
                /* 在换乘站[u]的等待时间（取自下一段的prev_wait） */
                if (prev_wait[w] > 0) {
                    char wn[MAX_NAME_LEN];
                    NODE_NAME(wn, u);
                    printf("    └ 在[%s]换乘等待 %d分钟\n", wn, prev_wait[w]);
                }
            }
            /* 开始新的一段 */
            merge_from = u;
            merge_time = prev_edge_time[w];
            merge_cost = prev_edge_cost[w];
            strncpy(merge_sched, prev_schedule[w], 19);
            merge_sched[19] = '\0';
            merge_type = prev_type[w];
        }
    }

    /* 输出最后一段 */
    if (merge_from >= 0) {
        char fn[MAX_NAME_LEN], tn[MAX_NAME_LEN];
        NODE_NAME(fn, merge_from);
        NODE_NAME(tn, path[0]);
        const char *ts = (merge_type == TRANSPORT_PLANE) ? "飞机" : "火车";
        printf("  乘坐%s[%s]  从[%s]到[%s]  耗时%d分钟，票价%d元\n",
               ts, merge_sched, fn, tn, merge_time, merge_cost);
    }

    #undef NODE_NAME
}

/* ================================================================
 * dijkstra：多起点、多终点、最短时间或最省费用
 *
 * 多起点：将所有起点同时以dist=0压入堆（等价于虚拟超级源点，但无需修改图）
 * 多终点：Dijkstra结束后，扫描所有终点候选，取dist最小的作为最终终点
 * ================================================================ */
static int dijkstra(int starts[], int ns, int ends[], int ne, int weight_type, int *out_dist) {
    int n = g.city_count + g.station_count;
    int *dist    = (int *)malloc(sizeof(int) * n);
    int *visited = (int *)calloc(n, sizeof(int));
    if (!dist || !visited) {
        fprintf(stderr, "[ERROR] Out of memory\n");
        free(dist); free(visited); return -1;
    }
    for (int i = 0; i < n; i++) { dist[i] = INT_MAX; prev_node[i] = -1; }

    MinHeap *heap = create_heap(n * 4);
    if (!heap) { free(dist); free(visited); return -1; }

    /* 多起点：全部以距离0入堆 */
    for (int i = 0; i < ns; i++) {
        dist[starts[i]] = 0;
        heap_push(heap, starts[i], 0);
    }

    while (heap->size > 0) {
        HeapNode cur = heap_pop(heap);
        int u = cur.node;
        if (visited[u]) continue;
        visited[u] = 1;

        for (Edge *e = g.adj[u]; e; e = e->next) {
            int v      = e->to_id;
            int weight, wait = 0;
            if (weight_type == 0) {
                /* 时间模式：纯行驶时间 + 中转等待时间 */
                if (e->depart_min >= 0 && dist[u] > 0) {
                    int tod = dist[u] % (24 * 60);
                    if (e->depart_min >= tod)
                        wait = e->depart_min - tod;
                    else
                        wait = e->depart_min + 24 * 60 - tod;
                }
                weight = wait + e->time_weight;
            } else if (weight_type == 1) {
                weight = e->cost_weight;
            } else {
                /* 换乘次数模式：同车次=0，换车次=1 */
                weight = (prev_node[u] == -1 ||
                          strcmp(prev_schedule[u], e->schedule_name) == 0) ? 0 : 1;
            }
            if (!visited[v] && dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev_node[v]      = u;
                prev_edge_time[v] = e->time_weight;
                prev_edge_cost[v] = e->cost_weight;
                prev_wait[v]      = wait;
                prev_type[v]      = e->type;
                strncpy(prev_schedule[v], e->schedule_name, 19);
                prev_schedule[v][19] = '\0';
                heap_push(heap, v, dist[v]);
            }
        }
    }

    /* 多终点：找dist最小的终点 */
    int best_end = -1, best_dist = INT_MAX;
    for (int i = 0; i < ne; i++) {
        if (dist[ends[i]] < best_dist) {
            best_dist = dist[ends[i]];
            best_end  = ends[i];
        }
    }

    if (out_dist) *out_dist = best_dist;

    free(dist); free(visited); free_heap(heap);
    return (best_end >= 0 && best_dist < INT_MAX) ? best_end : -1;
}

/* ================================================================
 * 辅助：节点ID → 名称和坐标
 * ================================================================ */
static void node_info(int id, char *name, double *lat, double *lon) {
    if (id < g.city_count) {
        strncpy(name, g.cities[id].name, MAX_NAME_LEN - 1);
        name[MAX_NAME_LEN - 1] = '\0';
        *lat = g.cities[id].lat;
        *lon = g.cities[id].lon;
    } else {
        int sid = id - g.city_count;
        strncpy(name, g.stations[sid].name, MAX_NAME_LEN - 1);
        name[MAX_NAME_LEN - 1] = '\0';
        if (g.stations[sid].has_coords) {
            *lat = g.stations[sid].lat;
            *lon = g.stations[sid].lon;
        } else {
            *lat = 0.0;
            *lon = 0.0;
        }
    }
}

/* ================================================================
 * print_result_text：交互模式的文本输出（原 dijkstra 中的打印逻辑）
 * ================================================================ */
static void print_result_text(int best_end, int best_dist, int weight_type) {
    const char *label = (weight_type == 0) ? "总时间（分钟）" :
                        (weight_type == 1) ? "总费用（元）" : "最少换乘次数";
    printf("%s: %d\n", label, best_dist);
    print_path(best_end);
}

/* ================================================================
 * print_result_json：JSON 格式输出路径结果
 * ================================================================ */
static void print_result_json(int best_end, int best_dist, int weight_type,
                              int start_ids[], int ns, int end_ids[], int ne) {
    /* 回溯路径，收集节点 */
    int path_nodes[MAX_NODES], path_len = 0;
    int v = best_end;
    while (v != -1 && path_len < MAX_NODES) {
        path_nodes[path_len++] = v;
        v = prev_node[v];
    }
    if (path_len >= MAX_NODES) {
        printf("{\"status\":\"error\",\"message\":\"路径过长，可能存在环\"}\n");
        return;
    }

    /* 输出 JSON */
    printf("{\"status\":\"ok\",\"total\":%d,", best_dist);
    const char *unit = (weight_type == 0) ? "分钟" :
                       (weight_type == 1) ? "元" : "次";
    printf("\"unit\":\"%s\",", unit);

    /* 起点候选 */
    printf("\"starts\":[");
    for (int i = 0; i < ns; i++) {
        char name[MAX_NAME_LEN]; double lat, lon;
        node_info(start_ids[i], name, &lat, &lon);
        printf("%s{\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f}",
               i > 0 ? "," : "", name, lat, lon);
    }
    printf("],");

    /* 终点候选 */
    printf("\"ends\":[");
    for (int i = 0; i < ne; i++) {
        char name[MAX_NAME_LEN]; double lat, lon;
        node_info(end_ids[i], name, &lat, &lon);
        printf("%s{\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f}",
               i > 0 ? "," : "", name, lat, lon);
    }
    printf("],");

    /* 路径段（合并同车次连续段） */
    printf("\"segments\":[");
    int seg_first = 1;
    int merge_from  = -1;
    int merge_time  = 0;
    int merge_cost  = 0;
    char merge_sched[20] = "";
    TransportType merge_type = TRANSPORT_TRAIN;

    for (int i = path_len - 1; i > 0; i--) {
        int u = path_nodes[i], w = path_nodes[i - 1];
        if (prev_edge_time[w] == 0 && prev_edge_cost[w] == 0) continue;

        int same = (merge_from >= 0 &&
                    strcmp(prev_schedule[w], merge_sched) == 0 &&
                    prev_type[w] == merge_type);

        if (same) {
            merge_time += prev_edge_time[w];
            merge_cost += prev_edge_cost[w];
        } else {
            if (merge_from >= 0) {
                char fn[MAX_NAME_LEN], tn[MAX_NAME_LEN];
                double flat, flon, tlat, tlon;
                node_info(merge_from, fn, &flat, &flon);
                node_info(u, tn, &tlat, &tlon);
                printf("%s{\"type\":\"%s\",\"schedule\":\"%s\","
                       "\"from\":\"%s\",\"from_lat\":%.6f,\"from_lon\":%.6f,"
                       "\"to\":\"%s\",\"to_lat\":%.6f,\"to_lon\":%.6f,"
                       "\"time\":%d,\"cost\":%d}",
                       seg_first ? "" : ",",
                       (merge_type == TRANSPORT_PLANE) ? "plane" : "train",
                       merge_sched, fn, flat, flon, tn, tlat, tlon,
                       merge_time, merge_cost);
                seg_first = 0;
            }
            merge_from = u;
            merge_time = prev_edge_time[w];
            merge_cost = prev_edge_cost[w];
            strncpy(merge_sched, prev_schedule[w], 19);
            merge_sched[19] = '\0';
            merge_type = prev_type[w];
        }
    }
    /* 最后一段 */
    if (merge_from >= 0) {
        char fn[MAX_NAME_LEN], tn[MAX_NAME_LEN];
        double flat, flon, tlat, tlon;
        node_info(merge_from, fn, &flat, &flon);
        node_info(path_nodes[0], tn, &tlat, &tlon);
        printf("%s{\"type\":\"%s\",\"schedule\":\"%s\","
               "\"from\":\"%s\",\"from_lat\":%.6f,\"from_lon\":%.6f,"
               "\"to\":\"%s\",\"to_lat\":%.6f,\"to_lon\":%.6f,"
               "\"time\":%d,\"cost\":%d}",
               seg_first ? "" : ",",
               (merge_type == TRANSPORT_PLANE) ? "plane" : "train",
               merge_sched, fn, flat, flon, tn, tlat, tlon,
               merge_time, merge_cost);
    }
    printf("]}\n");
}

/* ================================================================
 * cmd_query_json：命令行 --query 模式的入口
 * ================================================================ */
static void cmd_query_json(const char *start_name, const char *end_name,
                           int decision, TransportType tt) {
    int start_ids[MAX_CANDIDATES], end_ids[MAX_CANDIDATES];
    int ns = find_nodes(start_name, tt, start_ids, MAX_CANDIDATES);
    int ne = find_nodes(end_name,   tt, end_ids,   MAX_CANDIDATES);

    if (ns == 0) {
        printf("{\"status\":\"error\",\"message\":\"找不到起始地: %s\"}\n", start_name);
        return;
    }
    if (ne == 0) {
        printf("{\"status\":\"error\",\"message\":\"找不到目的地: %s\"}\n", end_name);
        return;
    }

    int weight_type = (decision == 1) ? 0 : (decision == 2) ? 1 : 2;
    int best_dist = 0;
    int best_end = dijkstra(start_ids, ns, end_ids, ne, weight_type, &best_dist);

    if (best_end < 0) {
        printf("{\"status\":\"error\",\"message\":\"无法到达！两点之间没有可用路线。\"}\n");
    } else {
        print_result_json(best_end, best_dist, weight_type,
                          start_ids, ns, end_ids, ne);
    }
}

/* ================================================================
 * cmd_list_stations：输出所有站点（JSON数组）
 * ================================================================ */
static void cmd_list_stations(void) {
    printf("{\"stations\":[");
    int first = 1;
    for (int i = 0; i < g.station_count; i++) {
        if (!g.stations[i].has_coords) continue;
        printf("%s{\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,\"city_id\":%d}",
               first ? "" : ",",
               g.stations[i].name,
               g.stations[i].lat, g.stations[i].lon,
               g.stations[i].city_id);
        first = 0;
    }
    printf("]}\n");
}

/* ================================================================
 * cmd_list_cities：输出所有城市（JSON数组）
 * ================================================================ */
static void cmd_list_cities(void) {
    printf("{\"cities\":[");
    for (int i = 0; i < g.city_count; i++) {
        printf("%s{\"name\":\"%s\",\"lat\":%.6f,\"lon\":%.6f}",
               i > 0 ? "," : "",
               g.cities[i].name,
               g.cities[i].lat, g.cities[i].lon);
    }
    printf("]}\n");
}

/* ================================================================
 * interactive_query：交互查询入口
 * 使用 find_nodes 获取候选站点列表，传入多起点/多终点算法
 * ================================================================ */
static void interactive_query(void) {
    char start_name[MAX_NAME_LEN], end_name[MAX_NAME_LEN];
    int decision, transport;

    printf("\n===== 交通咨询系统 =====\n");
    printf("请输入起始地（城市名或精确站名）：");
    if (scanf("%31s", start_name) != 1) return;
    printf("请输入目的地（城市名或精确站名）：");
    if (scanf("%31s", end_name) != 1) return;

    do {
        printf("决策原则 (1-最快到达  2-最省钱  3-最少中转)：");
        if (scanf("%d", &decision) != 1) {
            decision = 0;
            while (getchar() != '\n') {}   /* 清空输入缓冲区，避免死循环 */
        }
    } while (decision < 1 || decision > 3);

    do {
        printf("交通工具 (1-火车  2-飞机)：");
        if (scanf("%d", &transport) != 1) {
            transport = 0;
            while (getchar() != '\n') {}
        }
    } while (transport < 1 || transport > 2);

    TransportType tt = (transport == 1) ? TRANSPORT_TRAIN : TRANSPORT_PLANE;

    int start_ids[MAX_CANDIDATES], end_ids[MAX_CANDIDATES];
    int ns = find_nodes(start_name, tt, start_ids, MAX_CANDIDATES);
    int ne = find_nodes(end_name,   tt, end_ids,   MAX_CANDIDATES);

    if (ns == 0) { printf("错误：找不到起始地 \"%s\"\n", start_name); return; }
    if (ne == 0) { printf("错误：找不到目的地 \"%s\"\n", end_name);   return; }

    /* 打印实际使用的起终点（辅助调试） */
    printf("起点候选站（%d个）：", ns);
    for (int i = 0; i < ns; i++) {
        int id = start_ids[i];
        printf("[%s] ", id < g.city_count ? g.cities[id].name
                                          : g.stations[id-g.city_count].name);
    }
    printf("\n终点候选站（%d个）：", ne);
    for (int i = 0; i < ne; i++) {
        int id = end_ids[i];
        printf("[%s] ", id < g.city_count ? g.cities[id].name
                                          : g.stations[id-g.city_count].name);
    }
    printf("\n");

    int best_dist = 0;
    int best_end = -1;
    switch (decision) {
        case 1: best_end = dijkstra(start_ids, ns, end_ids, ne, 0, &best_dist); break;
        case 2: best_end = dijkstra(start_ids, ns, end_ids, ne, 1, &best_dist); break;
        case 3: best_end = dijkstra(start_ids, ns, end_ids, ne, 2, &best_dist); break;
    }
    if (best_end < 0) {
        printf("无法到达！两点之间没有可用路线。\n");
    } else {
        print_result_text(best_end, best_dist,
            (decision == 1) ? 0 : (decision == 2) ? 1 : 2);
    }
}

/* ================================================================
   主函数
   [F3] 只构建一次图，调用统一的 build_graph()
   ================================================================ */
int main(int argc, char **argv) {
#ifdef _WIN32
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
#endif

    /* ---- 命令行模式检测 ---- */
    int is_list_stations = 0, is_list_cities = 0, is_query = 0, is_query_file = 0;
    char q_from[MAX_NAME_LEN] = "", q_to[MAX_NAME_LEN] = "";
    int  q_mode = 0, q_transport = 0;
    char q_file_path[512] = "";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--list-stations") == 0)
            is_list_stations = 1;
        else if (strcmp(argv[i], "--list-cities") == 0)
            is_list_cities = 1;
        else if (strcmp(argv[i], "--query") == 0)
            is_query = 1;
        else if (strcmp(argv[i], "--query-file") == 0 && i + 1 < argc)
            { is_query_file = 1; strncpy(q_file_path, argv[++i], 511); q_file_path[511] = '\0'; }
        else if (strcmp(argv[i], "--from") == 0 && i + 1 < argc)
            strncpy(q_from, argv[++i], MAX_NAME_LEN - 1);
        else if (strcmp(argv[i], "--to") == 0 && i + 1 < argc)
            strncpy(q_to, argv[++i], MAX_NAME_LEN - 1);
        else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc)
            q_mode = atoi(argv[++i]);
        else if (strcmp(argv[i], "--transport") == 0 && i + 1 < argc)
            q_transport = atoi(argv[++i]);
    }

    if (is_list_stations || is_list_cities || is_query || is_query_file)
        g_json_mode = 1;

    /* 1~5. 加载所有数据 */
    load_cities_csv();
    load_airports_csv();
    load_flights_csv();
    load_stations_csv();
    load_railway_csv();

    LOG("实际加载城市：%d 个，站点：%d 个",
           g.city_count, g.station_count);

    /* 6. 构建图 */
    build_graph();

    /* 命令行模式：执行后直接退出 */
    if (is_list_stations) {
        cmd_list_stations();
        free_graph();
        return 0;
    }
    if (is_list_cities) {
        cmd_list_cities();
        free_graph();
        return 0;
    }
    if (is_query_file) {
        /* 从 UTF-8 文本文件读取查询参数：第1行=起点, 第2行=终点, 第3行=mode, 第4行=transport */
        FILE *qf = fopen(q_file_path, "rb");
        if (!qf) {
            printf("{\"status\":\"error\",\"message\":\"无法打开查询文件\"}\n");
        } else {
            char line1[MAX_NAME_LEN] = "", line2[MAX_NAME_LEN] = "";
            char line3[16] = "", line4[16] = "";
            if (fgets(line1, sizeof(line1), qf)) strip_line(line1);
            if (fgets(line2, sizeof(line2), qf)) strip_line(line2);
            if (fgets(line3, sizeof(line3), qf)) strip_line(line3);
            if (fgets(line4, sizeof(line4), qf)) strip_line(line4);
            fclose(qf);
            if (line1[0] && line2[0]) {
                int m = atoi(line3), t = atoi(line4);
                if (m < 1 || m > 3) m = 1;
                if (t < 1 || t > 2) t = 1;
                TransportType tt = (t == 1) ? TRANSPORT_TRAIN : TRANSPORT_PLANE;
                cmd_query_json(line1, line2, m, tt);
            } else {
                printf("{\"status\":\"error\",\"message\":\"查询文件格式错误\"}\n");
            }
        }
        free_graph();
        return 0;
    }
    if (is_query) {
        if (q_from[0] == '\0' || q_to[0] == '\0') {
            printf("{\"status\":\"error\",\"message\":\"缺少--from或--to参数\"}\n");
        } else if (q_mode < 1 || q_mode > 3) {
            printf("{\"status\":\"error\",\"message\":\"--mode必须为1(最快) 2(最省钱) 3(最少换乘)\"}\n");
        } else if (q_transport < 1 || q_transport > 2) {
            printf("{\"status\":\"error\",\"message\":\"--transport必须为1(火车) 2(飞机)\"}\n");
        } else {
            TransportType tt = (q_transport == 1) ? TRANSPORT_TRAIN : TRANSPORT_PLANE;
            cmd_query_json(q_from, q_to, q_mode, tt);
        }
        free_graph();
        return 0;
    }

    /* 交互模式 */
    LOG("==============================================");
    LOG(" Transport Planning System  v3.0  2025-05");
    LOG(" (no iconv, MAX_STATIONS=8000, with waypoints)");
    LOG("==============================================");

    char cont;
    do {
        interactive_query();
        printf("\n继续查询？(y/n)：");
        scanf(" %c", &cont);
    } while (cont == 'y' || cont == 'Y');

    free_graph();
    return 0;
}