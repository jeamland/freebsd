#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/capsicum.h>
#include <sys/domain.h>
#include <sys/lock.h>
#include <sys/kernel.h>
#include <sys/malloc.h>		/* XXX must be before <sys/file.h> */
#include <sys/mbuf.h>
#include <sys/msgbus.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/protosw.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/systm.h>

#include <vm/uma.h>

LIST_HEAD(mbp_head, msgbuspcb);

static uma_zone_t	msgbuspcb_zone;
static struct mbp_head	mbp_shead;

static void	msgbus_init(void);
static int	msgbus_externalize(struct mbuf *, struct mbuf **, int);

static struct domain msgbusdomain;
static struct pr_usrreqs msgbus_usrreqs;

#ifndef PIPSIZ
#define	PIPSIZ	8192
#endif
static u_long	msgbus_sendspace = PIPSIZ;	/* really max datagram size */
static u_long	msgbus_recvspace = PIPSIZ;

static uint64_t		msgbus_address = 0;
static struct mtx	msgbus_address_lock;
static struct mtx	msgbus_list_lock;

static struct protosw msgbussw[] = {
{
	.pr_type =		SOCK_SEQPACKET,
	.pr_domain =		&msgbusdomain,
	.pr_flags =		PR_ADDR|PR_ATOMIC|PR_WANTRCVD|PR_IMPLOPCL,
	.pr_usrreqs =		&msgbus_usrreqs,
},
};

static struct domain msgbusdomain = {
	.dom_family =		AF_MSGBUS,
	.dom_name =		"msg",
	.dom_init =		msgbus_init,
	.dom_externalize =	msgbus_externalize,
	.dom_protosw =		msgbussw,
	.dom_protoswNPROTOSW =	&msgbussw[nitems(msgbussw)]
};

DOMAIN_SET(msgbus);

/* Domain Methods */

#define	MSGBUS_LIST_LOCK_INIT()		mtx_init(&msgbus_list_lock,	\
					    "mbp_list_lock", NULL, MTX_DEF)
#define	MSGBUS_LIST_LOCK()			mtx_lock(&msgbus_list_lock)
#define	MSGBUS_LIST_UNLOCK()		mtx_unlock(&msgbus_list_lock)

#define	MSGBUS_ADDRESS_LOCK_INIT()	mtx_init(&msgbus_address_lock,	\
					    "mbp_address_lock", NULL, MTX_DEF)
#define	MSGBUS_ADDRESS_LOCK()			mtx_lock(&msgbus_address_lock)
#define	MSGBUS_ADDRESS_UNLOCK()		mtx_unlock(&msgbus_address_lock)

#define MSGBUS_PCB_LOCK_INIT(mbp)	mtx_init(&(mbp)->mbp_mtx,	\
					    "mbp_mtx", "mbp_mtx",	\
					    MTX_DUPOK|MTX_DEF|MTX_RECURSE)
#define	MSGBUS_PCB_LOCK_DESTROY(mbp)	mtx_destroy(&(mbp)->mbp_mtx)
#define	MSGBUS_PCB_LOCK(mbp)		mtx_lock(&(mbp)->mbp_mtx)
#define	MSGBUS_PCB_UNLOCK(mbp)		mtx_unlock(&(mbp)->mbp_mtx)
#define	MSGBUS_PCB_LOCK_ASSERT(mbp)	mtx_assert(&(mbp)->mbp_mtx, MA_OWNED)

static void
msgbus_init(void)
{

	msgbuspcb_zone = uma_zcreate("msgpcb", sizeof(struct msgbuspcb), NULL,
	    NULL, NULL, NULL, UMA_ALIGN_PTR, 0);
	if (msgbuspcb_zone == NULL)
		panic("msgbus_init");
	uma_zone_set_max(msgbuspcb_zone, maxsockets);
	uma_zone_set_warning(msgbuspcb_zone,
	    "kern.ipc.maxsockets limit reached");

	MSGBUS_LIST_LOCK_INIT();
	MSGBUS_ADDRESS_LOCK_INIT();
}

static int
msgbus_externalize(struct mbuf *control, struct mbuf **controlp, int flags)
{
	struct cmsghdr *cm = mtod(control, struct cmsghdr *);
	void *data;
	socklen_t clen = control->m_len, datalen;
	int error;

	error = 0;
	if (controlp != NULL) /* controlp == NULL => free control messages */
		*controlp = NULL;
	while (cm != NULL) {
		if (sizeof(*cm) > clen || cm->cmsg_len > clen) {
			error = EINVAL;
			break;
		}
		data = CMSG_DATA(cm);
		datalen = (caddr_t)cm + cm->cmsg_len - (caddr_t)data;
		if (error || controlp == NULL)
			goto next;
		*controlp = sbcreatecontrol(NULL, datalen,
		    cm->cmsg_type, cm->cmsg_level);
		if (*controlp == NULL) {
			error = ENOBUFS;
			goto next;
		}
		bcopy(data,
		    CMSG_DATA(mtod(*controlp, struct cmsghdr *)),
		    datalen);
		controlp = &(*controlp)->m_next;

next:
		if (CMSG_SPACE(datalen) < clen) {
			clen -= CMSG_SPACE(datalen);
			cm = (struct cmsghdr *)
			    ((caddr_t)cm + CMSG_SPACE(datalen));
		} else {
			clen = 0;
			cm = NULL;
		}
	}

	m_freem(control);
	return (error);
}

/* Protocol Methods */

static uint64_t
msgbus_next_address(void)
{
	uint64_t	address;

	MSGBUS_ADDRESS_LOCK();
	address = msgbus_address;
	msgbus_address++;
	MSGBUS_ADDRESS_UNLOCK();

	return (address);
}

static int
msgbus_attach(struct socket *so, int proto, struct thread *td)
{
	u_long sendspace, recvspace;
	struct msgbuspcb *mbp;
	int error;

	KASSERT(so->so_pcb == NULL, ("msgbus_attach: so_pcb != NULL"));
	if (so->so_snd.sb_hiwat == 0 || so->so_rcv.sb_hiwat == 0) {
		switch (so->so_type) {
		case SOCK_SEQPACKET:
			sendspace = msgbus_sendspace;
			recvspace = msgbus_recvspace;
			break;

		default:
			panic("msgbus_attach");
		}
		error = soreserve(so, sendspace, recvspace);
		if (error)
			return (error);
	}
	mbp = uma_zalloc(msgbuspcb_zone, M_NOWAIT | M_ZERO);
	if (mbp == NULL)
		return (ENOBUFS);
	MSGBUS_PCB_LOCK_INIT(mbp);
	mbp->mbp_socket = so;
	mbp->mbp_name.smb_len = sizeof(struct sockaddr_msgbus);
	mbp->mbp_name.smb_family = AF_MSGBUS;
	mbp->mbp_name.smb_address = msgbus_next_address();
	so->so_pcb = mbp;

	MSGBUS_LIST_LOCK();
	LIST_INSERT_HEAD(&mbp_shead, mbp, mbp_link);
	MSGBUS_LIST_UNLOCK();

	return (0);
}

static int
msgbus_bind(struct socket *so, struct sockaddr *nam, struct thread *td)
{

	return (EOPNOTSUPP);
}

static void
msgbus_detach(struct socket *so)
{
	struct msgbuspcb *mbp;

	mbp = sotomsgbuspcb(so);
	KASSERT(mbp != NULL, ("msgbus_detach: mbp == NULL"));

	MSGBUS_LIST_LOCK();
	LIST_REMOVE(mbp, mbp_link);
	MSGBUS_LIST_UNLOCK();

	mbp->mbp_socket->so_pcb = NULL;
	MSGBUS_PCB_LOCK_DESTROY(mbp);
	uma_zfree(msgbuspcb_zone, mbp);
}

static int
msgbus_send(struct socket *so, int flags, struct mbuf *m, struct sockaddr *nam,
    struct mbuf *control, struct thread *td)
{
	struct msgbuspcb *mbp, *mbp2;
	struct socket *so2;
	struct sockaddr *from;
	struct sockaddr_msgbus *to;
	struct sectoken *token;
	struct timeval *tv;
	u_int mbcnt, sbcc;
	int error = 0;

	if (nam == NULL) {
		error = EDESTADDRREQ;
		goto release;
	}
	if (nam->sa_family != AF_MSGBUS) {
		error = EAFNOSUPPORT;
		goto release;
	}
	to = (struct sockaddr_msgbus *)nam;

	mbp = sotomsgbuspcb(so);
	KASSERT(mbp != NULL, ("%s: mbp == NULL", __func__));
	KASSERT(so->so_type == SOCK_SEQPACKET,
	    ("%s: socktype %d", __func__, so->so_type));

	if (flags & PRUS_OOB) {
		error = EOPNOTSUPP;
		goto release;
	}
	if (control != NULL) {
		printf("msgbus_send: not set up for control messages yet\n");
		goto release;
	}

	/* Lockless read. XXX needs to check receive end instead */
	if (so->so_snd.sb_state & SBS_CANTSENDMORE) {
		error = EPIPE;
		goto release;
	}

	/* Build up security token. */
	control = sbcreatecontrol(NULL, sizeof(*token), SCM_SECTOKEN,
	    SOL_SOCKET);
	if (control == NULL) {
		error = ENOBUFS;
		goto release;
	}
	token = (struct sectoken *)
	    CMSG_DATA(mtod(control, struct cmsghdr *));
	token->st_pid = td->td_proc->p_pid;
	token->st_uid = td->td_ucred->cr_ruid;
	token->st_euid = td->td_ucred->cr_uid;

	/* Add timestamp. */
	control->m_next = sbcreatecontrol(NULL, sizeof(*tv),
	    SCM_TIMESTAMP, SOL_SOCKET);
	if (control->m_next == NULL) {
		error = ENOBUFS;
		goto release;
	}
	tv = (struct timeval *)
	    CMSG_DATA(mtod(control->m_next, struct cmsghdr *));
	microtime(tv);

	MSGBUS_LIST_LOCK();
	LIST_FOREACH(mbp2, &mbp_shead, mbp_link) {
		if (mbp2->mbp_address == to->smb_address)
			break;
	}
	MSGBUS_LIST_UNLOCK();

	if (mbp2 == NULL) {
		error = ENOENT;
		goto release;
	}

	so2 = mbp2->mbp_socket;
	MSGBUS_PCB_LOCK(mbp2);
	SOCKBUF_LOCK(&so2->so_rcv);

	from = (struct sockaddr *)&(mbp->mbp_name);
	/*
	 * Don't check for space available in so2->so_rcv.
	 * Unix domain sockets only check for space in the
	 * sending sockbuf, and that check is performed one
	 * level up the stack.
	 * XXX This isn't a UNIX socket.
	 */
	if (sbappendaddr_nospacecheck_locked(&so2->so_rcv, from, m, control))
		control = NULL;

	mbcnt = so2->so_rcv.sb_mbcnt;
	sbcc = sbavail(&so2->so_rcv);
	if (sbcc)
		sorwakeup_locked(so2);
	else
		SOCKBUF_UNLOCK(&so2->so_rcv);

	/*
	 * The PCB lock on mbp2 protects the SB_STOP flag.  Without it,
	 * it would be possible for uipc_rcvd to be called at this
	 * point, drain the receiving sockbuf, clear SB_STOP, and then
	 * we would set SB_STOP below.  That could lead to an empty
	 * sockbuf having SB_STOP set
	 */
	SOCKBUF_LOCK(&so->so_snd);
	if (sbcc >= so->so_snd.sb_hiwat || mbcnt >= so->so_snd.sb_mbmax)
		so->so_snd.sb_flags |= SB_STOP;
	SOCKBUF_UNLOCK(&so->so_snd);
	MSGBUS_PCB_UNLOCK(mbp2);
	m = NULL;

release:
	if (control != NULL)
		m_freem(control);
	if (m != NULL)
		m_freem(m);
	return (error);
}

static int
msgbus_sockaddr(struct socket *so, struct sockaddr **nam)
{
	struct msgbuspcb *mbp;
	struct sockaddr_msgbus *smb;

	mbp = sotomsgbuspcb(so);
	KASSERT(mbp != NULL, ("msgbus_sockaddr: mbp == NULL"));

	*nam = malloc(sizeof(struct sockaddr_msgbus), M_SONAME, M_WAITOK);
	smb = &(mbp->mbp_name);
	bcopy(smb, *nam, smb->smb_len);

	return (0);
}

static struct pr_usrreqs msgbus_usrreqs = {
	.pru_attach =	msgbus_attach,
	.pru_bind =	msgbus_bind,
	.pru_detach =	msgbus_detach,
	.pru_send =	msgbus_send,
	.pru_sockaddr =	msgbus_sockaddr,
};
