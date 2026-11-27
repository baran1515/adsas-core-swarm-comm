/**
 * @file node.c
 * @brief A.D.S.A.S. swarm node: TCP full-mesh, heartbeats, failure detection,
 *        coordinator-driven task re-allocation. POSIX (Linux) only.
 *
 * Each node listens on base_port + id and opens one outbound TCP connection to
 * every peer. Outbound sockets are used for sending, inbound sockets for
 * receiving; frames are recovered from the byte stream with adsas_parser_t.
 *
 * Position source:
 *   - default: built-in kinematic model (node "flies" its route at --speed)
 *   - --ext-port P: positions come from an external bridge (ArduPilot SITL)
 *     as UDP text "POS x_dm y_dm alt_dm"; the node answers with "ROUTE ...".
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "adsas/protocol.h"
#include "adsas/task_alloc.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define HB_PERIOD_MS        200u
#define HB_TIMEOUT_MS      1000u  /* 5 missed heartbeats => suspect      */
#define STARTUP_GRACE_MS   8000u  /* time to wait for never-seen peers   */
#define RECONNECT_MS       1000u
#define STATUS_MS          1000u
#define REACHED_DM           10   /* 1 m waypoint acceptance radius      */
#define MAX_INBOUND          32

typedef struct {
    int      out_fd;              /* -1 if not connected                 */
    int      seen;                /* received at least one heartbeat     */
    uint32_t last_hb_ms;
    adsas_heartbeat_t hb;         /* last heartbeat received             */
} peer_t;

typedef struct {
    int            fd;
    adsas_parser_t parser;
} inbound_t;

typedef struct {
    /* configuration */
    uint8_t  id;
    uint8_t  n_nodes;
    uint16_t base_port;
    char     peer_host[64];
    double   speed_mps;
    double   fail_at_s;          /* <0: never */
    double   jam_at_s;           /* <0: never */
    double   jam_for_s;
    int      ext_port;           /* 0: internal kinematics */
    const char *csv_path;

    /* state */
    adsas_area_t area;
    adsas_plan_t plan;
    peer_t       peers[ADSAS_MAX_NODES];
    inbound_t    inbound[MAX_INBOUND];
    int          listen_fd;
    int          ext_fd;
    struct sockaddr_in ext_peer;
    int          ext_peer_valid;
    uint16_t     suspect_mask;
    int          isolated;          /* hears no peer at all              */
    uint32_t     holdoff_until;     /* no evictions before this time     */
    uint32_t     epoch_ahead_since; /* 0: no peer is on a newer epoch    */
    uint16_t     seq;
    uint16_t     done;
    uint8_t      state;
    int32_t      x_dm, y_dm;
    int16_t      alt_dm;
    uint32_t     t0;
    FILE        *csv;
} node_t;

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static void logf_(node_t *n, const char *fmt, ...)
{
    va_list ap;
    printf("[%7.2fs][N%u][e%u] ", (now_ms() - n->t0) / 1000.0, n->id, n->plan.epoch);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
    fflush(stdout);
}

static int set_nonblock(int fd)
{
    const int fl = fcntl(fd, F_GETFL, 0);
    return fl < 0 ? -1 : fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static int is_jammed(const node_t *n)
{
    if (n->jam_at_s < 0) return 0;
    const double t = (now_ms() - n->t0) / 1000.0;
    return t >= n->jam_at_s && t < n->jam_at_s + n->jam_for_s;
}

/* ---- sockets ------------------------------------------------------------ */
static int open_listen(uint16_t port)
{
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    const int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0 || listen(fd, 16) < 0) {
        close(fd);
        return -1;
    }
    set_nonblock(fd);
    return fd;
}

static int connect_peer(const char *host, uint16_t port)
{
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port) };
    if (inet_pton(AF_INET, host, &a.sin_addr) != 1 ||
        connect(fd, (struct sockaddr *)&a, sizeof(a)) < 0) {
        close(fd);
        return -1;
    }
    const int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)); /* low latency */
    set_nonblock(fd);
    return fd;
}

static void send_all_peers(node_t *n, uint8_t type, const uint8_t *payload, uint16_t len)
{
    if (is_jammed(n)) return; /* simulated jamming: nothing leaves the radio */

    uint8_t frame[ADSAS_FRAME_MAX];
    const size_t fl = adsas_frame_build(frame, sizeof(frame), type, n->id, ADSAS_BROADCAST,
                                        n->seq++, now_ms() - n->t0, payload, len);
    for (unsigned id = 1; id <= n->n_nodes; ++id) {
        peer_t *p = &n->peers[id];
        if (id == n->id || p->out_fd < 0) continue;
        if (send(p->out_fd, frame, fl, MSG_NOSIGNAL) != (ssize_t)fl) {
            close(p->out_fd);
            p->out_fd = -1;
        }
    }
}

/* ---- protocol handlers -------------------------------------------------- */
static void publish_sync(node_t *n)
{
    adsas_task_sync_t ts = { n->plan.epoch, n->plan.alive_mask, adsas_plan_hash(&n->plan) };
    uint8_t buf[sizeof(ts)];
    send_all_peers(n, ADSAS_MSG_TASK_SYNC, buf, (uint16_t)adsas_task_sync_write(buf, &ts));
}

static void apply_node_down(node_t *n, uint8_t failed, uint16_t failed_done)
{
    const int moved = adsas_plan_reassign(&n->plan, &n->area, failed, failed_done);
    n->suspect_mask &= (uint16_t)~(1u << failed);
    logf_(n, "NODE %u DOWN -> %d cells re-allocated, alive=0x%04x, my route %u/%u",
          failed, moved, n->plan.alive_mask, n->done, n->plan.count[n->id]);
    if (n->state == ADSAS_STATE_IDLE && n->done < n->plan.count[n->id])
        n->state = ADSAS_STATE_MISSION; /* got new work after finishing */
}

static void handle_frame(node_t *n, const adsas_frame_t *f)
{
    const uint8_t src = f->hdr.src;
    if (src == 0 || src > n->n_nodes || src == n->id) return;

    switch (adsas_msg_type(&f->hdr)) {
    case ADSAS_MSG_HEARTBEAT: {
        adsas_heartbeat_t hb;
        if (adsas_heartbeat_read(f->payload, f->hdr.len, &hb) != 0) return;
        peer_t *p = &n->peers[src];
        p->hb = hb;
        p->seen = 1;
        p->last_hb_ms = now_ms();
        if (n->suspect_mask & (1u << src)) {
            n->suspect_mask &= (uint16_t)~(1u << src);
            logf_(n, "node %u heard again, suspicion cleared", src);
        }
        if (!(n->plan.alive_mask & (1u << src)))
            logf_(n, "node %u was removed from plan; re-join is future work", src);
        /* A peer persistently on a newer epoch means we missed NODE_DOWN
         * messages (e.g. we were jammed and the swarm removed us). */
        if (hb.epoch > n->plan.epoch && n->state != ADSAS_STATE_RTL) {
            if (!n->epoch_ahead_since) n->epoch_ahead_since = now_ms();
            else if (now_ms() - n->epoch_ahead_since > HB_TIMEOUT_MS) {
                n->state = ADSAS_STATE_RTL;
                logf_(n, "peers on epoch %u, I am on %u -> I was removed, RTL", hb.epoch, n->plan.epoch);
            }
        } else if (hb.epoch <= n->plan.epoch) {
            n->epoch_ahead_since = 0;
        }
        break;
    }
    case ADSAS_MSG_NODE_DOWN: {
        adsas_node_down_t nd;
        if (adsas_node_down_read(f->payload, f->hdr.len, &nd) != 0) return;
        if (nd.node_id == n->id) {
            n->state = ADSAS_STATE_RTL;
            logf_(n, "swarm declared ME down (link lost) -> RTL, mission handed over");
            return;
        }
        if (nd.new_epoch == (uint16_t)(n->plan.epoch + 1u))
            apply_node_down(n, nd.node_id, nd.cells_done);
        else if (nd.new_epoch > n->plan.epoch)
            logf_(n, "EPOCH GAP (have %u, got %u) - plan resync needed", n->plan.epoch, nd.new_epoch);
        break;
    }
    case ADSAS_MSG_TASK_SYNC: {
        adsas_task_sync_t ts;
        if (adsas_task_sync_read(f->payload, f->hdr.len, &ts) != 0) return;
        if (ts.epoch == n->plan.epoch) {
            const uint32_t mine = adsas_plan_hash(&n->plan);
            logf_(n, "plan sync from N%u: %s (hash %08x)", src,
                  mine == ts.plan_hash ? "CONSISTENT" : "MISMATCH", ts.plan_hash);
        }
        break;
    }
    default:
        break;
    }
}

/* ---- periodic tasks ----------------------------------------------------- */
static void send_heartbeat(node_t *n)
{
    adsas_heartbeat_t hb = {
        .state = n->state, .battery = 100, .x_dm = n->x_dm, .y_dm = n->y_dm,
        .alt_dm = n->alt_dm, .epoch = n->plan.epoch, .cells_done = n->done,
        .flags = (uint8_t)(n->ext_port ? 0x02 : 0x00),
    };
    uint8_t buf[sizeof(hb)];
    send_all_peers(n, ADSAS_MSG_HEARTBEAT, buf, (uint16_t)adsas_heartbeat_write(buf, &hb));
}

static void detect_failures(node_t *n)
{
    const uint32_t t = now_ms();
    for (unsigned id = 1; id <= n->n_nodes; ++id) {
        if (id == n->id || !(n->plan.alive_mask & (1u << id))) continue;
        const peer_t *p = &n->peers[id];
        const int timed_out = p->seen ? (t - p->last_hb_ms > HB_TIMEOUT_MS)
                                      : (t - n->t0 > STARTUP_GRACE_MS);
        if (timed_out && !(n->suspect_mask & (1u << id))) {
            n->suspect_mask |= (uint16_t)(1u << id);
            logf_(n, "node %u SUSPECT (no heartbeat for %u ms)", id,
                  p->seen ? t - p->last_hb_ms : t - n->t0);
        }
    }

    /* Isolation guard: if every other alive node is suspect, the problem is
     * most likely OUR link (we are the jammed one). Never evict the whole
     * swarm; keep flying the own route autonomously instead. */
    const uint16_t others = (uint16_t)(n->plan.alive_mask & ~(1u << n->id));
    const int isolated = others && (n->suspect_mask & others) == others;
    if (isolated != n->isolated) {
        n->isolated = isolated;
        if (!isolated) n->holdoff_until = t + 2u * HB_TIMEOUT_MS; /* let the view settle */
        logf_(n, isolated ? "ISOLATED: no peer heard, continuing own route autonomously"
                          : "link to swarm restored");
    }
    if (isolated || (int32_t)(t - n->holdoff_until) < 0) return;

    /* Only a node that currently hears at least one other peer may evict
     * anybody: a node whose own receiver is jammed hears nobody. */
    int heard_recent = 0;
    for (unsigned id = 1; id <= n->n_nodes; ++id)
        if (id != n->id && n->peers[id].seen && t - n->peers[id].last_hb_ms <= 2u * HB_PERIOD_MS)
            heard_recent = 1;
    if (!heard_recent) return;

    /* Coordinator = lowest id that is alive and not suspected. Only it
     * publishes NODE_DOWN, which keeps the re-allocation order identical. */
    const int coord = adsas_coordinator((uint16_t)(n->plan.alive_mask & ~n->suspect_mask));
    if (coord != n->id || is_jammed(n)) return;

    for (unsigned id = 1; id <= n->n_nodes; ++id) {
        if (!(n->suspect_mask & (1u << id)) || !(n->plan.alive_mask & (1u << id))) continue;
        const peer_t *p = &n->peers[id];
        adsas_node_down_t nd = {
            .node_id = (uint8_t)id, .reason = ADSAS_DOWN_TIMEOUT,
            .new_epoch = (uint16_t)(n->plan.epoch + 1u),
            .cells_done = p->seen ? p->hb.cells_done : 0,
            .last_seen_ms = p->seen ? p->last_hb_ms - n->t0 : 0,
        };
        uint8_t buf[sizeof(nd)];
        send_all_peers(n, ADSAS_MSG_NODE_DOWN, buf, (uint16_t)adsas_node_down_write(buf, &nd));
        logf_(n, "I am coordinator: announcing node %u down", id);
        apply_node_down(n, (uint8_t)id, nd.cells_done);
        publish_sync(n);
    }
}

static void update_kinematics(node_t *n, double dt)
{
    if (n->state != ADSAS_STATE_MISSION) return;
    if (n->done >= n->plan.count[n->id]) {
        n->state = ADSAS_STATE_IDLE;
        logf_(n, "route complete (%u cells)", n->done);
        return;
    }
    const adsas_point_t tgt = adsas_cell_center(&n->area, n->plan.route[n->id][n->done]);
    const double dx = tgt.x_dm - n->x_dm, dy = tgt.y_dm - n->y_dm;
    const double d = sqrt(dx * dx + dy * dy);

    if (!n->ext_port) {
        const double step = n->speed_mps * 10.0 * dt; /* dm this tick */
        if (d <= step) { n->x_dm = tgt.x_dm; n->y_dm = tgt.y_dm; }
        else { n->x_dm += (int32_t)lround(dx / d * step); n->y_dm += (int32_t)lround(dy / d * step); }
    }
    if (d <= REACHED_DM || (!n->ext_port && n->x_dm == tgt.x_dm && n->y_dm == tgt.y_dm))
        n->done++;
}

static void ext_send_route(node_t *n)
{
    if (!n->ext_peer_valid) return;
    char msg[1400];
    int k = snprintf(msg, sizeof(msg), "ROUTE %u %u %u %u", n->plan.epoch, n->state, n->done,
                     n->plan.count[n->id]);
    for (uint16_t i = n->done; i < n->plan.count[n->id] && k < (int)sizeof(msg) - 32; ++i) {
        const adsas_point_t p = adsas_cell_center(&n->area, n->plan.route[n->id][i]);
        k += snprintf(msg + k, sizeof(msg) - (size_t)k, " %d,%d", p.x_dm, p.y_dm);
    }
    msg[k++] = '\n';
    sendto(n->ext_fd, msg, (size_t)k, 0, (struct sockaddr *)&n->ext_peer, sizeof(n->ext_peer));
}

static void ext_poll(node_t *n)
{
    char buf[256];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    ssize_t r;
    while ((r = recvfrom(n->ext_fd, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &fl)) > 0) {
        buf[r] = '\0';
        int x, y, alt;
        if (sscanf(buf, "POS %d %d %d", &x, &y, &alt) == 3) {
            n->x_dm = x; n->y_dm = y; n->alt_dm = (int16_t)alt;
            n->ext_peer = from;
            n->ext_peer_valid = 1;
        }
        fl = sizeof(from);
    }
}

static void csv_row(node_t *n)
{
    if (!n->csv) return;
    fprintf(n->csv, "%.2f,%u,%u,%u,%d,%d,%u,%u,%u\n", (now_ms() - n->t0) / 1000.0, n->id,
            n->plan.epoch, n->state, n->x_dm, n->y_dm, n->done, n->plan.count[n->id],
            n->plan.alive_mask);
    fflush(n->csv);
}

/* ---- main loop ---------------------------------------------------------- */
static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s --id N --nodes K [options]\n"
        "  --base-port P   TCP base port (node i listens on P+i)   [47000]\n"
        "  --host IP       peer host address                      [127.0.0.1]\n"
        "  --grid CxR      coverage grid                            [8x8]\n"
        "  --cell M        cell edge in metres                      [20]\n"
        "  --speed V       kinematic speed m/s                      [8]\n"
        "  --fail-at S     process exits at S seconds (node lost)\n"
        "  --jam-at S      stop transmitting at S seconds (jamming)\n"
        "  --jam-for S     jamming duration                         [30]\n"
        "  --ext-port P    UDP port for SITL bridge positions\n"
        "  --csv FILE      write state log as CSV\n", argv0);
}

int main(int argc, char **argv)
{
    node_t n;
    memset(&n, 0, sizeof(n));
    n.base_port = 47000;
    strcpy(n.peer_host, "127.0.0.1");
    n.speed_mps = 8.0;
    n.fail_at_s = n.jam_at_s = -1.0;
    n.jam_for_s = 30.0;
    n.area = (adsas_area_t){ .cols = 8, .rows = 8, .cell_dm = 200, .origin_x_dm = 0, .origin_y_dm = 0 };

    static const struct option opts[] = {
        { "id", required_argument, 0, 'i' },        { "nodes", required_argument, 0, 'n' },
        { "base-port", required_argument, 0, 'p' }, { "host", required_argument, 0, 'H' },
        { "grid", required_argument, 0, 'g' },      { "cell", required_argument, 0, 'c' },
        { "speed", required_argument, 0, 's' },     { "fail-at", required_argument, 0, 'f' },
        { "jam-at", required_argument, 0, 'j' },    { "jam-for", required_argument, 0, 'J' },
        { "ext-port", required_argument, 0, 'e' },  { "csv", required_argument, 0, 'o' },
        { 0, 0, 0, 0 }
    };
    int c, cols = 8, rows = 8;
    while ((c = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        switch (c) {
        case 'i': n.id = (uint8_t)atoi(optarg); break;
        case 'n': n.n_nodes = (uint8_t)atoi(optarg); break;
        case 'p': n.base_port = (uint16_t)atoi(optarg); break;
        case 'H': snprintf(n.peer_host, sizeof(n.peer_host), "%s", optarg); break;
        case 'g': if (sscanf(optarg, "%dx%d", &cols, &rows) != 2) { usage(argv[0]); return 2; } break;
        case 'c': n.area.cell_dm = (int32_t)(atof(optarg) * 10.0); break;
        case 's': n.speed_mps = atof(optarg); break;
        case 'f': n.fail_at_s = atof(optarg); break;
        case 'j': n.jam_at_s = atof(optarg); break;
        case 'J': n.jam_for_s = atof(optarg); break;
        case 'e': n.ext_port = atoi(optarg); break;
        case 'o': n.csv_path = optarg; break;
        default: usage(argv[0]); return 2;
        }
    }
    if (n.id < 1 || n.n_nodes < 2 || n.id > n.n_nodes || n.n_nodes >= ADSAS_MAX_NODES ||
        cols < 1 || rows < 1 || cols * rows > (int)ADSAS_MAX_CELLS) {
        usage(argv[0]);
        return 2;
    }
    n.area.cols = (uint8_t)cols;
    n.area.rows = (uint8_t)rows;

    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    uint16_t mask = 0;
    for (unsigned id = 1; id <= n.n_nodes; ++id) mask |= (uint16_t)(1u << id);
    adsas_plan_initial(&n.plan, &n.area, mask);

    for (unsigned i = 0; i < ADSAS_MAX_NODES; ++i) n.peers[i].out_fd = -1;
    for (unsigned i = 0; i < MAX_INBOUND; ++i) n.inbound[i].fd = -1;

    n.listen_fd = open_listen((uint16_t)(n.base_port + n.id));
    if (n.listen_fd < 0) { perror("listen"); return 1; }

    if (n.ext_port) {
        n.ext_fd = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons((uint16_t)n.ext_port),
                                 .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
        if (n.ext_fd < 0 || bind(n.ext_fd, (struct sockaddr *)&a, sizeof(a)) < 0) {
            perror("ext-port");
            return 1;
        }
        set_nonblock(n.ext_fd);
    }
    if (n.csv_path && (n.csv = fopen(n.csv_path, "w")) != NULL)
        fprintf(n.csv, "t_s,id,epoch,state,x_dm,y_dm,done,total,alive_mask\n");

    /* launch line south of the area, 3 m apart */
    n.x_dm = n.area.origin_x_dm + (int32_t)n.id * 30;
    n.y_dm = n.area.origin_y_dm - 100;
    n.alt_dm = 200;
    n.state = ADSAS_STATE_MISSION;
    n.t0 = now_ms();
    n.holdoff_until = n.t0;

    logf_(&n, "up: %u nodes, grid %ux%u, cell %.1f m, my route %u cells, hash %08x",
          n.n_nodes, n.area.cols, n.area.rows, n.area.cell_dm / 10.0, n.plan.count[n.id],
          adsas_plan_hash(&n.plan));

    uint32_t last_hb = 0, last_conn = 0, last_status = 0, last_tick = now_ms();
    int jam_logged = 0;

    while (!g_stop) {
        struct pollfd pfd[1 + MAX_INBOUND];
        int np = 0;
        pfd[np++] = (struct pollfd){ .fd = n.listen_fd, .events = POLLIN };
        for (int i = 0; i < MAX_INBOUND; ++i)
            if (n.inbound[i].fd >= 0) pfd[np++] = (struct pollfd){ .fd = n.inbound[i].fd, .events = POLLIN };
        poll(pfd, (nfds_t)np, 20);

        const uint32_t t = now_ms();
        const double ts = (t - n.t0) / 1000.0;
        if (n.fail_at_s >= 0 && ts >= n.fail_at_s) {
            logf_(&n, "SIMULATED LOSS: node goes silent permanently");
            break;
        }
        if (is_jammed(&n) && !jam_logged) { logf_(&n, "JAMMED: transmitter blocked"); jam_logged = 1; }
        if (!is_jammed(&n) && jam_logged) { logf_(&n, "jamming ended"); jam_logged = 0; }

        /* accept */
        int fd;
        while ((fd = accept(n.listen_fd, NULL, NULL)) >= 0) {
            set_nonblock(fd);
            int slot = -1;
            for (int i = 0; i < MAX_INBOUND; ++i) if (n.inbound[i].fd < 0) { slot = i; break; }
            if (slot < 0) { close(fd); continue; }
            n.inbound[slot].fd = fd;
            adsas_parser_init(&n.inbound[slot].parser);
        }

        /* receive */
        for (int i = 0; i < MAX_INBOUND; ++i) {
            inbound_t *in = &n.inbound[i];
            if (in->fd < 0) continue;
            uint8_t buf[512];
            const ssize_t r = recv(in->fd, buf, sizeof(buf), 0);
            if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                close(in->fd);
                in->fd = -1;
                continue;
            }
            if (is_jammed(&n)) continue; /* receiver also blinded */
            adsas_frame_t f;
            for (ssize_t k = 0; k < r; ++k)
                if (adsas_parser_push(&in->parser, buf[k], &f)) handle_frame(&n, &f);
        }

        if (n.ext_port) ext_poll(&n);

        if (t - last_conn >= RECONNECT_MS) {
            last_conn = t;
            for (unsigned id = 1; id <= n.n_nodes; ++id)
                if (id != n.id && n.peers[id].out_fd < 0 && (n.plan.alive_mask & (1u << id)))
                    n.peers[id].out_fd = connect_peer(n.peer_host, (uint16_t)(n.base_port + id));
        }
        if (t - last_hb >= HB_PERIOD_MS) {
            last_hb = t;
            send_heartbeat(&n);
            if (n.state != ADSAS_STATE_RTL) detect_failures(&n);
        }

        const uint16_t prev_done = n.done, prev_epoch = n.plan.epoch;
        update_kinematics(&n, (t - last_tick) / 1000.0);
        last_tick = t;
        if (n.ext_port && (n.done != prev_done || n.plan.epoch != prev_epoch)) ext_send_route(&n);

        if (t - last_status >= STATUS_MS) {
            last_status = t;
            csv_row(&n);
            if (n.ext_port) ext_send_route(&n);
        }
    }

    logf_(&n, "shutdown (route %u/%u)", n.done, n.plan.count[n.id]);
    if (n.csv) fclose(n.csv);
    return 0;
}
