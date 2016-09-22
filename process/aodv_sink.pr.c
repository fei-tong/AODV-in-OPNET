/* Process model C form file: aodv_sink.pr.c */
/* Portions of this file copyright 1986-2008 by OPNET Technologies, Inc. */



/* This variable carries the header into the object file */
const char aodv_sink_pr_c [] = "MIL_3_Tfile_Hdr_ 145A 30A modeler 7 4CD0A9D0 4CD0A9D0 1 china-0f9728557 Administrator 0 0 none none 0 0 none 0 0 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                            ";
#include <string.h>



/* OPNET system definitions */
#include <opnet.h>



/* Header Block */

#include <stdio.h>

#define IN_STRM		0
#define PKT_RCVD	(op_intrpt_type () == OPC_INTRPT_STRM)
#define END_SIM		(op_intrpt_type () == OPC_INTRPT_ENDSIM)

typedef struct
	{
	int own_input;
	int forward_input;
	int own_output;
	int forward_output;
	int data_pk_destroyed;
	int data_pk_buffer;
	int ack;
	} StatBlock;

extern StatBlock stats[50];
extern int data_pk_buffer;
extern int data_pk_destroyed;
extern int output;
extern int input;
extern int ack;

int subm_pkts;
int rcv;
int hop_number;
int rcv_control;
double total_power;
double control_data_power;
int total_hop_received;
FILE* out;
int pk_in_queue;
int Replyfromtarget;
int Replyfromrelay;
int GratuitousReply;
int RouteError;
int totaldata;
int dataserved;
int data_count;
int request_created;
int non_propa_request;
int renew_request;
int renew_request_propa;
int request_from_error;
int error;
int error_to_source;
int packets_extracted;
int packets_to_serve;
int num_spare_route;
int amazing_error;
int canceled_reply;

int mac_total_retry;
int mac_failed;
int mac_failed_data;
int mac_failed_reply;
int mac_failed_error;
int mac_amazing_error;
int mac_retry_rts;
int mac_retry_data;
int mac_retry_reply;
int mac_retry_error;
extern int getBufferSize(int);




/* End of Header Block */

#if !defined (VOSD_NO_FIN)
#undef	BIN
#undef	BOUT
#define	BIN		FIN_LOCAL_FIELD(_op_last_line_passed) = __LINE__ - _op_block_origin;
#define	BOUT	BIN
#define	BINIT	FIN_LOCAL_FIELD(_op_last_line_passed) = 0; _op_block_origin = __LINE__;
#else
#define	BINIT
#endif /* #if !defined (VOSD_NO_FIN) */



/* State variable definitions */
typedef struct
	{
	/* Internal state tracking for FSM */
	FSM_SYS_STATE
	/* State Variables */
	Objid	                  		process_id                                      ;
	char	                   		outfile_name[256]                               ;
	int	                    		node_addr                                       ;
	int	                    		DEBUG                                           ;
	int	                    		node_id                                         ;
	int	                    		num_nodes                                       ;
	int	                    		pk_num                                          ;
	} aodv_sink_state;

#define process_id              		op_sv_ptr->process_id
#define outfile_name            		op_sv_ptr->outfile_name
#define node_addr               		op_sv_ptr->node_addr
#define DEBUG                   		op_sv_ptr->DEBUG
#define node_id                 		op_sv_ptr->node_id
#define num_nodes               		op_sv_ptr->num_nodes
#define pk_num                  		op_sv_ptr->pk_num

/* These macro definitions will define a local variable called	*/
/* "op_sv_ptr" in each function containing a FIN statement.	*/
/* This variable points to the state variable data structure,	*/
/* and can be used from a C debugger to display their values.	*/
#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE
#define FIN_PREAMBLE_DEC	aodv_sink_state *op_sv_ptr;
#define FIN_PREAMBLE_CODE	\
		op_sv_ptr = ((aodv_sink_state *)(OP_SIM_CONTEXT_PTR->_op_mod_state_ptr));


/* Function Block */

#if !defined (VOSD_NO_FIN)
enum { _op_block_origin = __LINE__ + 2};
#endif

void proc_pkt (void)
	{
	Packet* pkptr;
	char    file_name[256];
	/*
	/* Destroyed the upcoming packet from the lower layer
	*/
	FIN(proc_pkt (void));
	// Extract packet from the upcoming stream
	pkptr = op_pk_get (IN_STRM);
	
	pk_num++;
	op_ima_obj_attr_get(process_id,"Sink File Two",file_name);
	out = fopen(file_name,"w");
	fprintf(out,"当前节点：%d. 收到数据包个数: %d.\n",node_addr,pk_num);
	fclose(out);
	// Detroy it
	op_pk_destroy (pkptr);	
	FOUT;
	}

void record_stats(void)
	{
	int i;
	
	/*
	/* write the stats into the outputfile
	*/
	FIN(record_stats(void));
	out = fopen(outfile_name,"w");
	fprintf (out,"GLOBAL STATS ::\n");
	fprintf (out,"~~~~~~~~~~~~~~~\n");
	fprintf (out,"	input : %d\n", input);
	fprintf (out,"	output : %d\n", output);
	fprintf (out,"	data packet destroyed because of non entry: %d\n", data_pk_destroyed);
	fprintf (out,"	data packet still in buffer: %d\n", data_pk_buffer);
	fprintf (out,"	Ack : %d\n", ack);
	fprintf (out,"\nINDIVIDUAL STATS ::\n");
	fprintf (out,"~~~~~~~~~~~~~~~~~~~\n");
	
	for(i=0;i< num_nodes;i++)
		{
			fprintf (out,"\n-> Node %d ::\n",i);
			fprintf (out,"-------------\n");
			fprintf (out,"	--------------------\n");		
			fprintf (out,"	| TOTAL INPUT: %d  \n",stats[i].own_input+stats[i].forward_input );
			fprintf (out,"	--------------------\n");		
			fprintf (out,"	own_input : %d\n",stats[i].own_input);
			fprintf (out,"	forward_input : %d\n\n", stats[i].forward_input);
			fprintf (out,"	--------------------\n");
			fprintf (out,"	| TOTAL OUTPUT: %d  \n",stats[i].own_output+stats[i].forward_output );
			fprintf (out,"	--------------------\n");
			fprintf (out,"	own_output : %d\n",stats[i].own_output);
		   	fprintf (out,"	forward_output : %d\n\n", stats[i].forward_output);
			fprintf (out,"	data packet destroyed : %d\n", stats[i].data_pk_destroyed);
			fprintf (out,"	data packet still in buffer: %d\n", stats[i].data_pk_buffer);
			fprintf (out,"	Ack : %d\n", stats[i].ack);

		}

	fclose(out);
	FOUT;
	}

/* End of Function Block */

/* Undefine optional tracing in FIN/FOUT/FRET */
/* The FSM has its own tracing code and the other */
/* functions should not have any tracing.		  */
#undef FIN_TRACING
#define FIN_TRACING

#undef FOUTRET_TRACING
#define FOUTRET_TRACING

#if defined (__cplusplus)
extern "C" {
#endif
	void aodv_sink (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Obtype _op_aodv_sink_init (int * init_block_ptr);
	void _op_aodv_sink_diag (OP_SIM_CONTEXT_ARG_OPT);
	void _op_aodv_sink_terminate (OP_SIM_CONTEXT_ARG_OPT);
	VosT_Address _op_aodv_sink_alloc (VosT_Obtype, int);
	void _op_aodv_sink_svar (void *, const char *, void **);


#if defined (__cplusplus)
} /* end of 'extern "C"' */
#endif




/* Process model interrupt handling procedure */


void
aodv_sink (OP_SIM_CONTEXT_ARG_OPT)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	FIN_MT (aodv_sink ());

		{


		FSM_ENTER ("aodv_sink")

		FSM_BLOCK_SWITCH
			{
			/*---------------------------------------------------------*/
			/** state (init) enter executives **/
			FSM_STATE_ENTER_FORCED_NOLABEL (0, "init", "aodv_sink [init enter execs]")
				FSM_PROFILE_SECTION_IN ("aodv_sink [init enter execs]", state0_enter_exec)
				{
				/* Initialize different state variables */
				process_id = (op_id_self());
				node_id = op_topo_parent (process_id);
				/* Read node's address */
				op_ima_obj_attr_get(node_id,"MAC Address",&node_addr);
				op_ima_obj_attr_get(node_id,"DEBUG",&DEBUG);
				/* Copy the name of the out[ut file */ 
				op_ima_obj_attr_get(process_id,"Sink File",outfile_name);
				//sprintf(outfile_name,"output_file");
				/* Read the number of nodes in the network */
				num_nodes = op_topo_object_count (OPC_OBJTYPE_NDMOB);
				pk_num=0;
				}
				FSM_PROFILE_SECTION_OUT (state0_enter_exec)

			/** state (init) exit executives **/
			FSM_STATE_EXIT_FORCED (0, "init", "aodv_sink [init exit execs]")


			/** state (init) transition processing **/
			FSM_TRANSIT_FORCE (1, state1_enter_exec, ;, "default", "", "init", "idle", "tr_10", "aodv_sink [init -> idle : default / ]")
				/*---------------------------------------------------------*/



			/** state (idle) enter executives **/
			FSM_STATE_ENTER_UNFORCED (1, "idle", state1_enter_exec, "aodv_sink [idle enter execs]")

			/** blocking after enter executives of unforced state. **/
			FSM_EXIT (3,"aodv_sink")


			/** state (idle) exit executives **/
			FSM_STATE_EXIT_UNFORCED (1, "idle", "aodv_sink [idle exit execs]")


			/** state (idle) transition processing **/
			FSM_PROFILE_SECTION_IN ("aodv_sink [idle trans conditions]", state1_trans_conds)
			FSM_INIT_COND (PKT_RCVD)
			FSM_TEST_COND (END_SIM)
			FSM_DFLT_COND
			FSM_TEST_LOGIC ("idle")
			FSM_PROFILE_SECTION_OUT (state1_trans_conds)

			FSM_TRANSIT_SWITCH
				{
				FSM_CASE_TRANSIT (0, 1, state1_enter_exec, proc_pkt();, "PKT_RCVD", "proc_pkt()", "idle", "idle", "tr_2", "aodv_sink [idle -> idle : PKT_RCVD / proc_pkt()]")
				FSM_CASE_TRANSIT (1, 1, state1_enter_exec, record_stats();, "END_SIM", "record_stats()", "idle", "idle", "tr_4", "aodv_sink [idle -> idle : END_SIM / record_stats()]")
				FSM_CASE_TRANSIT (2, 1, state1_enter_exec, ;, "default", "", "idle", "idle", "tr_3", "aodv_sink [idle -> idle : default / ]")
				}
				/*---------------------------------------------------------*/



			}


		FSM_EXIT (0,"aodv_sink")
		}
	}




void
_op_aodv_sink_diag (OP_SIM_CONTEXT_ARG_OPT)
	{
	/* No Diagnostic Block */
	}




void
_op_aodv_sink_terminate (OP_SIM_CONTEXT_ARG_OPT)
	{

	FIN_MT (_op_aodv_sink_terminate ())


	/* No Termination Block */

	Vos_Poolmem_Dealloc (op_sv_ptr);

	FOUT
	}


/* Undefine shortcuts to state variables to avoid */
/* syntax error in direct access to fields of */
/* local variable prs_ptr in _op_aodv_sink_svar function. */
#undef process_id
#undef outfile_name
#undef node_addr
#undef DEBUG
#undef node_id
#undef num_nodes
#undef pk_num

#undef FIN_PREAMBLE_DEC
#undef FIN_PREAMBLE_CODE

#define FIN_PREAMBLE_DEC
#define FIN_PREAMBLE_CODE

VosT_Obtype
_op_aodv_sink_init (int * init_block_ptr)
	{
	VosT_Obtype obtype = OPC_NIL;
	FIN_MT (_op_aodv_sink_init (init_block_ptr))

	obtype = Vos_Define_Object_Prstate ("proc state vars (aodv_sink)",
		sizeof (aodv_sink_state));
	*init_block_ptr = 0;

	FRET (obtype)
	}

VosT_Address
_op_aodv_sink_alloc (VosT_Obtype obtype, int init_block)
	{
#if !defined (VOSD_NO_FIN)
	int _op_block_origin = 0;
#endif
	aodv_sink_state * ptr;
	FIN_MT (_op_aodv_sink_alloc (obtype))

	ptr = (aodv_sink_state *)Vos_Alloc_Object (obtype);
	if (ptr != OPC_NIL)
		{
		ptr->_op_current_block = init_block;
#if defined (OPD_ALLOW_ODB)
		ptr->_op_current_state = "aodv_sink [init enter execs]";
#endif
		}
	FRET ((VosT_Address)ptr)
	}



void
_op_aodv_sink_svar (void * gen_ptr, const char * var_name, void ** var_p_ptr)
	{
	aodv_sink_state		*prs_ptr;

	FIN_MT (_op_aodv_sink_svar (gen_ptr, var_name, var_p_ptr))

	if (var_name == OPC_NIL)
		{
		*var_p_ptr = (void *)OPC_NIL;
		FOUT
		}
	prs_ptr = (aodv_sink_state *)gen_ptr;

	if (strcmp ("process_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->process_id);
		FOUT
		}
	if (strcmp ("outfile_name" , var_name) == 0)
		{
		*var_p_ptr = (void *) (prs_ptr->outfile_name);
		FOUT
		}
	if (strcmp ("node_addr" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_addr);
		FOUT
		}
	if (strcmp ("DEBUG" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->DEBUG);
		FOUT
		}
	if (strcmp ("node_id" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->node_id);
		FOUT
		}
	if (strcmp ("num_nodes" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->num_nodes);
		FOUT
		}
	if (strcmp ("pk_num" , var_name) == 0)
		{
		*var_p_ptr = (void *) (&prs_ptr->pk_num);
		FOUT
		}
	*var_p_ptr = (void *)OPC_NIL;

	FOUT
	}

