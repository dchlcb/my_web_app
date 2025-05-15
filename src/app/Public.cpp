#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <error.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "Public.h"


/* 简易封装：type = F_RDLCK / F_WRLCK / F_UNLCK */
static int file_lock(int fd, short type)
{
    struct flock fl = 
    {
        .l_type   = type,
        .l_whence = SEEK_SET,
        .l_start  = 0,
        .l_len    = 0           /* 0 = EOF */
    };

    while (fcntl(fd, F_SETLKW, &fl) == -1) 
    {
        if (errno == EINTR) continue;
        perror("fcntl(F_SETLKW)");
        return -1;
    }
    return 0;
}


/********************************************************************
 * 写 / 更新坐标行：identifier  x  y  z
 * - 若文件已存在该 identifier 则覆盖其行
 * - 否则在文件尾追加一行
 ********************************************************************/
int write_coordinates(const char *identifier, float x, float y, float z)
{
    /* ------------------ 变量声明区 ------------------ */
    int      retval         = -1;    /* 默认失败 */
    int      fd             = -1;    /* 文件描述符 */
    FILE    *fp             = NULL;  /* 流式 FILE* 对象 */
    char    *line           = NULL;  /* getline 动态缓冲 */
    char    *tail_buf       = NULL;  /* 备份尾部数据 */
    size_t   linecap        = 0;     /* getline 缓冲长度 */
    size_t   tail_size      = 0;     /* 备份尾部字节数 */
    off_t    file_size      = -1;    /* 文件总大小 */
    off_t    hit_offset     = -1;    /* 目标行在文件中的偏移 */
    off_t    current_offset = 0;     /* getline 遍历时累计偏移 */
    size_t   hit_length     = 0;     /* 目标行长度(含 '\n') */
    char     newline[BUFFER_SIZE];   /* 新行缓冲 */
    int      nbytes         = 0;     /* 新行实际长度 */

    /* 1. 以读写方式打开文件；若不存在则创建 */
    fd = open(FILE_PATH, O_RDWR | O_CREAT, 0666);
    if (fd == -1) { perror("open"); goto cleanup; }

    /* 2. 加独占写锁，防止其他进程并发写 */
    if (file_lock(fd, F_WRLCK) == -1) goto cleanup;

    /* 3. 获取文件大小，然后把文件指针退回开头 */
    file_size = lseek(fd, 0, SEEK_END);
    if (file_size == -1) { perror("lseek"); goto cleanup; }
    if (lseek(fd, 0, SEEK_SET) == -1) { perror("lseek"); goto cleanup; }

    /* 4. fd → FILE*，方便逐行读取（不再 dup，避免泄漏） */
    fp = fdopen(fd, "r+");                       /* r+ 与 O_RDWR 匹配 */
    if (!fp) { perror("fdopen"); goto cleanup; }

    /* 5. getline 搜索目标 identifier */
    while (getline(&line, &linecap, fp) != -1)
    {
        size_t len = strlen(line);
        if (strncmp(line, identifier, strlen(identifier)) == 0 &&
            (line[strlen(identifier)] == ' ' || line[strlen(identifier)] == '\n'))
        {
            hit_offset = current_offset;        /* 行首偏移 */
            hit_length = len;                   /* 包含 '\n' */
            break;
        }
        current_offset += len;                  /* 下一行偏移 */
    }

    /* 6. 备份命中行后的尾部 */
    if (hit_offset != -1)
    {
        off_t tail_pos = hit_offset + hit_length;
        tail_size = (size_t)(file_size - tail_pos);
        if (tail_size)
        {
            tail_buf = (char*)malloc(tail_size);
            if (!tail_buf) { perror("malloc"); goto cleanup; }
            if (pread(fd, tail_buf, tail_size, tail_pos) != (ssize_t)tail_size)
            {
                perror("pread"); goto cleanup;
            }
        }
    }
    else
    {
        hit_offset = file_size;                 /* 追加 */
    }

    /* 7. 生成新的坐标行 */
    nbytes = snprintf(newline, sizeof(newline),
                      "%s %.1f %.1f %.1f\n", identifier, x, y, z);
    if (nbytes <= 0 || nbytes >= (int)sizeof(newline))
    {
        fprintf(stderr, "newline overflow\n"); goto cleanup;
    }

    /* -------------- 关键同步：fflush + fileno ---------------- */
    fflush(fp);                    /* 清空 FILE* 读写缓冲区       */
    fd = fileno(fp);               /* 重新取得内部 fd（同一描述符）*/

    /* 8. 截断 → 写入新行 + 尾部 */
    if (ftruncate(fd, hit_offset) == -1) { perror("ftruncate"); goto cleanup; }
    if (lseek(fd, hit_offset, SEEK_SET) == -1) { perror("lseek"); goto cleanup; }

    if (write(fd, newline, nbytes) != nbytes) { perror("write"); goto cleanup; }
    if (tail_size && write(fd, tail_buf, tail_size) != (ssize_t)tail_size)
    {
        perror("write tail"); goto cleanup;
    }

    /* 再次同步流对象状态，否则 stdio 的文件位置与真实位置不一致 */
    fflush(fp);                    /* 没有写缓冲，但可刷新 stdio 元数据 */
    fsync(fd);                     /* 强制落盘 */
    fseek(fp, 0, SEEK_END);        /* 更新 FILE* 内部 filepos */

    retval = 0;                    /* 成功 */

cleanup:
    if (line)     free(line);
    if (tail_buf) free(tail_buf);
    if (fp)
    {
        file_lock(fileno(fp), F_UNLCK);         /* 解锁 */
        fclose(fp);                             /* fclose 同时 close(fd) */
        fd = -1;
    }
    if (fd != -1) { file_lock(fd, F_UNLCK); close(fd); }
    return retval;
}

#if 1
/********************************************************************
 * 读取坐标行：identifier  x  y  z
 *   - 成功返回 0，并写入 out_x/out_y/out_z
 *   - 未找到或错误返回 -1
 ********************************************************************/
int read_coordinates(const char *identifier, float *out_x, float *out_y, float *out_z)
{
    int     retval   = -1;
    int     fd       = -1;
    FILE   *fp       = NULL;
    char   *line     = NULL;
    size_t  linecap  = 0;

    /* 1. 只读打开文件 */
    fd = open(FILE_PATH, O_RDONLY);
    if (fd == -1) { perror("open"); goto cleanup; }

    /* 2. 共享读锁 */
    if (file_lock(fd, F_RDLCK) == -1) goto cleanup;

    /* 3. 转成 FILE*，逐行读取 */
    fp = fdopen(fd, "r");
    if (!fp) { perror("fdopen"); goto cleanup; }

    while (getline(&line, &linecap, fp) != -1)
    {
        /* ---------- 去掉结尾的 \n / \r ---------- */
        size_t len = strlen(line);
        while (len && (line[len-1] == '\n' || line[len-1] == '\r'))
        line[--len] = '\0';

        /* ---------- 从尾部解析 z y x ---------- */
        char *p = line;
        char *endz = strrchr(p, ' ');  if (!endz) continue;
        *endz = '\0';
        float z = strtof(endz + 1, NULL);

        char *endy = strrchr(p, ' ');  if (!endy) continue;
        *endy = '\0';
        float y = strtof(endy + 1, NULL);

        char *endx = strrchr(p, ' ');  if (!endx) continue;
        *endx = '\0';
        float x = strtof(endx + 1, NULL);

        /* ---------- 剩余部分是 identifier ---------- */
        char *id_begin = p;
        while (isspace((unsigned char)*id_begin)) ++id_begin;
        char *id_end = id_begin + strlen(id_begin);
        while (id_end > id_begin && isspace((unsigned char)id_end[-1])) --id_end;
        *id_end = '\0';

        if (strcmp(id_begin, identifier) == 0) 
        {
            if (out_x) *out_x = x;
            if (out_y) *out_y = y;
            if (out_z) *out_z = z;
            retval = 0;
            break;
        }
    }

cleanup:
/* ---------- 收尾 ---------- */
    if (line) free(line);

    if (fp) 
    {                         /* fp 创建成功 */
        file_lock(fileno(fp), F_UNLCK);
        fclose(fp);                   /* fclose 会顺带 close(fd) */
        fd = -1;
    }
    if (fd != -1) 
    {                   /* 只在 fp 失败时才走 */
        file_lock(fd, F_UNLCK);
        close(fd);
    }
return retval;
}

#else
int read_coordinates(const char *identifier, float *out_x, float *out_y, float *out_z)
{
    int retval = -1;
    int fd = -1;
    FILE *fp = NULL;
    char *line = NULL;
    size_t linecap = 0;

    /* 1. 打开文件 (只读) */
    fd = open(FILE_PATH, O_RDONLY);
    if (fd == -1) { perror("open"); goto cleanup; }

    /* 2. 共享读锁 */
    if (file_lock(fd, F_RDLCK) == -1) goto cleanup;

    /* 3. 逐行读取 */
    if ((fp = fdopen(fd, "r")) == NULL) { perror("fdopen"); goto cleanup; }

    while (getline(&line, &linecap, fp) != -1) 
    {

        /* ---- 从尾部依次解析 z, y, x ---- */
        size_t len = strlen(line);
        if (len && line[len-1] == '\n') line[len-1] = '\0';   /* 去掉换行 */

        char *p = line;
        char *endz = strrchr(p, ' ');
        if (!endz) continue;
        *endz = '\0';                         /* 断开 z */
        float z = strtof(endz + 1, NULL);

        char *endy = strrchr(p, ' ');
        if (!endy) continue;
        *endy = '\0';                         /* 断开 y */
        float y = strtof(endy + 1, NULL);

        char *endx = strrchr(p, ' ');
        if (!endx) continue;
        *endx = '\0';                         /* 断开 x */
        float x = strtof(endx + 1, NULL);

        /* 剩余部分就是 identifier（去掉首尾空格） */
        char *id_begin = p;
        while (isspace((unsigned char)*id_begin)) ++id_begin;
        char *id_end = id_begin + strlen(id_begin);
        while (id_end > id_begin && isspace((unsigned char)id_end[-1])) --id_end;
        *id_end = '\0';

        if (strcmp(id_begin, identifier) == 0) 
        {
            if (out_x) *out_x = x;
            if (out_y) *out_y = y;
            if (out_z) *out_z = z;
            retval = 0;
            break;
        }
    }

    cleanup:
    /* -------- 正确的收尾顺序 -------- */
    if (fp) 
    {                                 // fp 创建成功
        file_lock(fileno(fp), F_UNLCK);       // ① 先解锁
        fclose(fp);                           // ② fclose 会顺带 close(fd)
        fd = -1;                              // ③ 防止后面重复 close
    }

    if (fd != -1) 
    {                           // 如果 fp 失败但 fd 打开
        file_lock(fd, F_UNLCK);
        close(fd);
    }

    return retval;
}
#endif

//合并两个字符串，并剔除其中的空格
void merge_and_remove_spaces(const char *str1, const char *str2, char **result) 
{
    // 计算两个字符串的长度
    size_t len1 = strlen(str1);
    size_t len2 = strlen(str2);

    // 分配合并后的字符串所需的内存（包括 '\0'）
    char *merged = (char *)malloc(len1 + len2 + 1);
    if (!merged) 
    {
        perror("malloc failed");
    }

    // 合并两个字符串
    strcpy(merged, str1);
    strcat(merged, str2);

    // 移除空格
    char *no_space = (char *)malloc(len1 + len2 + 1);
    if (!no_space) 
    {
        free(merged);
        perror("malloc failed");
    }

    char *write_ptr = no_space;
    for (const char *read_ptr = merged; *read_ptr != '\0'; ++read_ptr)
    {
        if (!isspace((unsigned char)*read_ptr)) 
        {
            *write_ptr++ = *read_ptr;
        }
    }
    *write_ptr = '\0'; // 末尾添加 '\0'

    // 释放中间合并的字符串
    free(merged);

    // 返回最终字符串
    *result = no_space;
}


int write_key_value_internal(const char *file_path, const char *key, const std::string &value)
{
    int retval = -1, fd = -1;
    FILE *fp = NULL;
    char *line = NULL, *tail_buf = NULL;
    size_t linecap = 0, tail_size = 0;
    off_t file_size = -1, hit_offset = -1, current_offset = 0;
    size_t hit_length = 0;
    char newline[256];
    int nbytes;

    /* 1. 打开并加独占写锁 */
    fd = open(file_path, O_RDWR | O_CREAT, 0666);
    if (fd == -1) { perror("open"); goto cleanup; }
    if (file_lock(fd, F_WRLCK) == -1) goto cleanup;

    /* 2. 文件大小 → 回到开头 */
    file_size = lseek(fd, 0, SEEK_END);
    if (file_size == -1) { perror("lseek"); goto cleanup; }
    if (lseek(fd, 0, SEEK_SET) == -1) { perror("lseek"); goto cleanup; }

    /* 3. 找到 key 所在行 */
    fp = fdopen(fd, "r");
    if (!fp) { perror("fdopen"); goto cleanup; }

    while (getline(&line, &linecap, fp) != -1) 
    {
        size_t len = strlen(line);
        if (strncmp(line, key, strlen(key)) == 0 && (line[strlen(key)] == ' ' || line[strlen(key)] == '\n')) 
        {
            hit_offset = current_offset;
            hit_length = len;
            break;
        }
        current_offset += len;
    }
    fclose(fp); fp = NULL;

    /* 4. 备份尾部 */
    if (hit_offset != -1) 
    {
        off_t tail_pos = hit_offset + hit_length;
        tail_size = static_cast<size_t>(file_size - tail_pos);
        if (tail_size) 
        {
            tail_buf = static_cast<char*>(malloc(tail_size));
            if (!tail_buf) { perror("malloc"); goto cleanup; }
            if (pread(fd, tail_buf, tail_size, tail_pos) != (ssize_t)tail_size) 
            {
                perror("pread"); goto cleanup;
            }
        }
    } 
    else 
    {
        hit_offset = file_size;      /* 追加模式 */
    }

    /* 5. 构造新行 */
   
    nbytes = snprintf(newline, sizeof(newline), "%s %s\n", key, value.c_str());
    if (nbytes <= 0 || nbytes >= (int)sizeof(newline)) 
    {
        fprintf(stderr, "newline overflow\n"); goto cleanup;
    }

    /* 6. 截断并写入 */
    if (ftruncate(fd, hit_offset) == -1) 
    { 
        perror("ftruncate"); goto cleanup; 
    }

    if (lseek(fd, hit_offset, SEEK_SET) == -1) 
    { 
        perror("lseek"); goto cleanup; 
    }

    if (write(fd, newline, nbytes) != nbytes) { perror("write"); goto cleanup; }
    if (tail_size && write(fd, tail_buf, tail_size) != (ssize_t)tail_size) 
    {
        perror("write tail"); goto cleanup;
    }
    if (fsync(fd) == -1) { perror("fsync"); goto cleanup; }

    retval = 0;

    cleanup:
    if (line)     free(line);
    if (tail_buf) free(tail_buf);
    if (fp)       fclose(fp);
    if (fd != -1) { file_lock(fd, F_UNLCK); close(fd); }
    return retval;
}


int read_key_value1(const char *file_path, const char *key, std::string &out_val)
{
    int fd = -1, retval = -1;
    FILE *fp = NULL;
    char *line = NULL;
    size_t linecap = 0;
    bool   locked = false;             /* ★ 锁是否成功标志 */
    size_t klen;

    /* ---- 打开文件 ---- */
    if ((fd = open(file_path, O_RDONLY)) == -1) 
    {
        perror("open"); goto cleanup;
    }

    /* ---- 共享读锁 ---- */
    if (file_lock(fd, F_RDLCK) == -1) goto cleanup;
    locked = true;                     /* ★ 只有成功才置 true */

    /* ---- 转成 FILE* ---- */
    if ((fp = fdopen(fd, "r")) == NULL) 
    {
        perror("fdopen"); goto cleanup;
    }

    /* ---- 扫描行 ---- */
    klen = strlen(key);
    while (getline(&line, &linecap, fp) != -1) 
    {
        if (strncmp(line, key, klen) != 0)          continue;
        if (!(line[klen] == ' ' || line[klen] == '\t')) continue;

        char *val = line + klen;
        while (*val == ' ' || *val == '\t') ++val;
        if (char *nl = strchr(val, '\n')) *nl = '\0';

        out_val = val;
        retval  = 0;
        break;
    }

    cleanup:
    if (line) free(line);

    /* ---- 正确收尾顺序 ---- */
    if (fp) 
    {
        if (locked) file_lock(fileno(fp), F_UNLCK);
        fclose(fp);            /* 同时关闭 fd */
        locked = false;
        fd = -1;
    }

    if (fd != -1) 
    {            /* 只有在 fp 创建失败时才走这里 */
        if (locked) file_lock(fd, F_UNLCK);
        close(fd);
    }

    return retval;
}