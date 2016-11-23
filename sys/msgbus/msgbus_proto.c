#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/capsicum.h>
#include <sys/domain.h>
#include <sys/lock.h>
#include <sys/kernel.h>
#include <sys/msgbus.h>
#include <sys/mutex.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/systm.h>

#include <vm/uma.h>

static uma_zone_t	msgbuspcb_zone;

static void	msgbus_init(void);

static struct domain msgbusdomain;
static struct pr_usrreqs msgbus_usrreqs;

#define MSGBUS_PCB_LOCK_INIT(mbp)	mtx_init(&(mbp)->mbp_mtx,	\
					    "mbp_mtx", "mbp_mtx",	\
					    MTX_DUPOK|MTX_DEF|MTX_RECURSE)
#define	MSGBUS_PCB_LOCK_DESTROY(mbp)	mtx_destroy(&(mbp)->mbp_mtx)

#ifndef PIPSIZ
#define	PIPSIZ	8192
#endif
static u_long	msgbus_sendspace = PIPSIZ;	/* really max datagram size */
static u_long	msgbus_recvspace = PIPSIZ;

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
	.dom_protosw =		msgbussw,
	.dom_protoswNPROTOSW =	&msgbussw[nitems(msgbussw)]
};

DOMAIN_SET(msgbus);

static void
msgbus_init(void)
{

	printf("I HAVE ARISEN\n");
	msgbuspcb_zone = uma_zcreate("msgpcb", sizeof(struct msgbuspcb), NULL,
	    NULL, NULL, NULL, UMA_ALIGN_PTR, 0);
	if (msgbuspcb_zone == NULL)
		panic("msgbus_init");
	uma_zone_set_max(msgbuspcb_zone, maxsockets);
	uma_zone_set_warning(msgbuspcb_zone,
	    "kern.ipc.maxsockets limit reached");
}

static int
msgbus_attach(struct socket *so, int proto, struct thread *td)
{
	u_long sendspace, recvspace;
	struct msgbuspcb *mbp;
	int error;

	printf("ATTACHMENT HAS BEEN MADE\n");

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
	so->so_pcb = mbp;

	return (0);
}

static void
msgbus_detach(struct socket *so)
{
	struct msgbuspcb *mbp;

	printf("DETACHMENT\n");

	mbp = sotomsgbuspcb(so);
	KASSERT(mbp != NULL, ("msgbus_detach: mbp == NULL"));

	mbp->mbp_socket->so_pcb = NULL;
	MSGBUS_PCB_LOCK_DESTROY(mbp);
	uma_zfree(msgbuspcb_zone, mbp);
}

static struct pr_usrreqs msgbus_usrreqs = {
	.pru_attach = msgbus_attach,
	.pru_detach = msgbus_detach,
};

