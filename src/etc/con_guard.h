#pragma once

#include <cstddef>
#include <cstring>
#include <strings.h>

// Conf key that holds a secret: contains pass, psk, key, secret, token or cred (any case).
inline bool isSecretKey(const char *k, size_t n) {
    static const char *const pats[] = {"pass", "psk", "key", "secret", "token", "cred"};
    for (auto p: pats)
        for (size_t m = strlen(p), i = 0; i + m <= n; ++i)
            if (!strncasecmp(k + i, p, m)) return true;
    return false;
}

inline bool isSecretKey(const char *k) { return isSecretKey(k, strlen(k)); }

static inline bool cgSpace(char c) { return c == ' ' || c == '\t'; }

// Offset where the secret part of a console or conf line starts (print s[0..cut) + "***"), -1 if none.
// `key=v` with a secret key; `wifi-add ssid:psk`; `scheme://user:pw@`; after a secret-key word (`set-config f psk v`,
// `token=v`).
// Mirrored in etc/fugu/console.py mask_secrets() and conf-editor.html maskSecrets().
inline int secretCut(const char *s) {
    int cut = -1;
    const char *k = s;
    while (*k == '#' || cgSpace(*k)) ++k;
    const char *ke = k;
    while (*ke && *ke != '=' && !cgSpace(*ke)) ++ke;
    const char *q = ke;
    while (cgSpace(*q)) ++q;
    if (*q == '=' && isSecretKey(k, ke - k)) return (int) (q + 1 - s);

    for (const char *p = s; *p; ++p)
        if (!strncasecmp(p, "wifi-add", 8)) {
            if (const char *c = strchr(p + 8, ':')) cut = (int) (c + 1 - s);
            break;
        }
    if (const char *u = strstr(s, "://"))
        if (strchr(u + 3, '@') && (cut < 0 || u + 3 - s < cut)) cut = (int) (u + 3 - s);

    const char *p = s;
    for (int w = 0; *p; ++w) {
        while (cgSpace(*p)) ++p;
        const char *b = p;
        while (*p && !cgSpace(*p)) ++p;
        if (b == p || !w) continue;
        const char *eq = (const char *) memchr(b, '=', p - b);
        if (!isSecretKey(b, (eq ? eq : p) - b)) continue;
        const char *v = eq ? eq + 1 : p;
        while (!eq && cgSpace(*v)) ++v;
        if (*v && (cut < 0 || v - s < cut)) cut = (int) (v - s);
        break;
    }
    return cut;
}

// Commands an untrusted console may run. Entry: registered name (with aliases), alone = no arguments,
// or `|*` = any arguments, or `|` and the comma-separated allowed values of a single argument (empty = none).
inline bool roCmdAllowed(const char *const *list, size_t n, const char *name, int nargs, const char *arg0) {
    const size_t nl = strlen(name);
    for (size_t i = 0; i < n; ++i) {
        const char *e = list[i];
        if (strncmp(e, name, nl) || (e[nl] && e[nl] != '|')) continue;
        if (!e[nl]) return !nargs;
        const char *a = e + nl + 1;
        if (!strcmp(a, "*")) return true;
        if (nargs > 1) return false;
        const char *v = nargs ? arg0 : "";
        const size_t vl = strlen(v);
        for (;; ++a) {
            const size_t il = strcspn(a, ",");
            if (il == vl && !strncmp(a, v, il)) return true;
            if (!a[il]) return false;
            a += il;
        }
    }
    return false;
}

// Read-only commands, the only ones an untrusted console (telnet unless trusted=1, BLE with ble_security none) may run.
inline constexpr const char *kRoCmds[] = {
    "help,?", "status", "sensor|*", "bootinfo", "uptime", "mem", "heap|,check,tasks", "tasks", "rt-stats", "intr",
    "hostname,hn", "ip", "netstat,ifconfig", "service,svc|,list", "coredump|,info", "selftest", "wire", "holiday",
    "balance", "sweep-curve", "rect-cal|status", "dt,deadtime", "pwm-freq,fsw", "pwm-dump", "rtprof",
    "get-config,getc|*", "conf-hash,confhash", "conf-check,confcheck", "ls|*", "cat|*", "scripts", "ble|,bonds",
};

// printf args for "%.*s%s": s up to `cut` (secretCut), then "***"
#define CG_MASKED(s, cut) ((cut) < 0 ? (int) strlen(s) : (cut)), (s), ((cut) < 0 ? "" : "***")
