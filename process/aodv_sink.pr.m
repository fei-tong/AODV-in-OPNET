MIL_3_Tfile_Hdr_ 145A 140A modeler 9 4CCE1AC3 4CD0A9D0 8 china-0f9728557 Administrator 0 0 none none 0 0 none 67C2FD72 1B36 0 0 0 0 0 0 1e80 8                                                                                                                                                                                                                                                                                                                                                                                  ЋЭg      @      Ю  в  Й  Р  Ф  Ш    ё  .  2  ­      	Sink File   џџџџџџџ   џџџџ       џџџџ      џџџџ      џџџџ           ЅZ             Sink File Two   џџџџџџџ   џџџџ       џџџџ      џџџџ      џџџџ           ЅZ                 	   begsim intrpt         
   џџџџ   
   doc file            	nd_module      endsim intrpt         
   џџџџ   
   failure intrpts         
   disabled   
   intrpt interval         
дВI­%У}џџџџ   
   priority              џџџџ      recovery intrpts            disabled      subqueue                     count    џџџ   
   џџџџ   
      list   	џџџ   
          
      super priority             џџџџ             Objid	\process_id;       char	\outfile_name[256];       int	\node_addr;       int	\DEBUG;       int	\node_id;       int	\num_nodes;       int	\pk_num;           C   #include <stdio.h>       #define IN_STRM		0   7#define PKT_RCVD	(op_intrpt_type () == OPC_INTRPT_STRM)   9#define END_SIM		(op_intrpt_type () == OPC_INTRPT_ENDSIM)       typedef struct   	{   	int own_input;   	int forward_input;   	int own_output;   	int forward_output;   	int data_pk_destroyed;   	int data_pk_buffer;   		int ack;   	} StatBlock;       extern StatBlock stats[50];   extern int data_pk_buffer;   extern int data_pk_destroyed;   extern int output;   extern int input;   extern int ack;       int subm_pkts;   int rcv;   int hop_number;   int rcv_control;   double total_power;   double control_data_power;   int total_hop_received;   
FILE* out;   int pk_in_queue;   int Replyfromtarget;   int Replyfromrelay;   int GratuitousReply;   int RouteError;   int totaldata;   int dataserved;   int data_count;   int request_created;   int non_propa_request;   int renew_request;   int renew_request_propa;   int request_from_error;   
int error;   int error_to_source;   int packets_extracted;   int packets_to_serve;   int num_spare_route;   int amazing_error;   int canceled_reply;       int mac_total_retry;   int mac_failed;   int mac_failed_data;   int mac_failed_reply;   int mac_failed_error;   int mac_amazing_error;   int mac_retry_rts;   int mac_retry_data;   int mac_retry_reply;   int mac_retry_error;   extern int getBufferSize(int);               ?   void proc_pkt (void)   	{   	Packet* pkptr;   	char    file_name[256];   	/*   6	/* Destroyed the upcoming packet from the lower layer   	*/   	FIN(proc_pkt (void));   +	// Extract packet from the upcoming stream   	pkptr = op_pk_get (IN_STRM);   	   
	pk_num++;   ;	op_ima_obj_attr_get(process_id,"Sink File Two",file_name);   	out = fopen(file_name,"w");   E	fprintf(out,"ЕБЧАНкЕуЃК%d. ЪеЕНЪ§ОнАќИіЪ§: %d.\n",node_addr,pk_num);   	fclose(out);   	// Detroy it   	op_pk_destroy (pkptr);	   	FOUT;   	}       void record_stats(void)   	{   	int i;   	   	/*   '	/* write the stats into the outputfile   	*/   	FIN(record_stats(void));   	out = fopen(outfile_name,"w");   #	fprintf (out,"GLOBAL STATS ::\n");   #	fprintf (out,"~~~~~~~~~~~~~~~\n");   &	fprintf (out,"	input : %d\n", input);   (	fprintf (out,"	output : %d\n", output);   V	fprintf (out,"	data packet destroyed because of non entry: %d\n", data_pk_destroyed);   D	fprintf (out,"	data packet still in buffer: %d\n", data_pk_buffer);   "	fprintf (out,"	Ack : %d\n", ack);   )	fprintf (out,"\nINDIVIDUAL STATS ::\n");   '	fprintf (out,"~~~~~~~~~~~~~~~~~~~\n");   	   	for(i=0;i< num_nodes;i++)   		{   '			fprintf (out,"\n-> Node %d ::\n",i);   #			fprintf (out,"-------------\n");   -			fprintf (out,"	--------------------\n");		   U			fprintf (out,"	| TOTAL INPUT: %d  \n",stats[i].own_input+stats[i].forward_input );   -			fprintf (out,"	--------------------\n");		   8			fprintf (out,"	own_input : %d\n",stats[i].own_input);   C			fprintf (out,"	forward_input : %d\n\n", stats[i].forward_input);   +			fprintf (out,"	--------------------\n");   X			fprintf (out,"	| TOTAL OUTPUT: %d  \n",stats[i].own_output+stats[i].forward_output );   +			fprintf (out,"	--------------------\n");   :			fprintf (out,"	own_output : %d\n",stats[i].own_output);   H		   	fprintf (out,"	forward_output : %d\n\n", stats[i].forward_output);   M			fprintf (out,"	data packet destroyed : %d\n", stats[i].data_pk_destroyed);   O			fprintf (out,"	data packet still in buffer: %d\n", stats[i].data_pk_buffer);   -			fprintf (out,"	Ack : %d\n", stats[i].ack);       		}       	fclose(out);   	FOUT;   	}                                           в          
   init   
       
      */* Initialize different state variables */   process_id = (op_id_self());   &node_id = op_topo_parent (process_id);   /* Read node's address */   6op_ima_obj_attr_get(node_id,"MAC Address",&node_addr);   ,op_ima_obj_attr_get(node_id,"DEBUG",&DEBUG);   '/* Copy the name of the out[ut file */    9op_ima_obj_attr_get(process_id,"Sink File",outfile_name);   &//sprintf(outfile_name,"output_file");   -/* Read the number of nodes in the network */   5num_nodes = op_topo_object_count (OPC_OBJTYPE_NDMOB);   	pk_num=0;   
                     
   џџџџ   
          pr_state        Т   в          
   idle   
       
       
       
       
           џџџџ             pr_state                     м   ]     Л   Е  Ъ   j  в   К             tr_2          
   PKT_RCVD   
       
   
proc_pkt()   
           џџџџ             џџџџ                       pr_transition              7   Ц     н   Ч     Ч  м   н             tr_3          
   default   
       џџџџ              џџџџ             џџџџ                       pr_transition              §  =     Й   ь  Ы  1  Я   ъ             tr_4          
   END_SIM   
       
   record_stats()   
           џџџџ             џџџџ                       pr_transition      
         -   в      Ћ   в  Ў   в          
   tr_10   
       џџџџ          џџџџ          
    џџџџ   
          џџџџ                       pr_transition                       	ETE delay        џџџџ   normal   linear        дВI­%У}   packet throughput        џџџџ   normal   linear        дВI­%У}   transmitted power        џџџџ   normal   linear        дВI­%У}      fifo   oms_string_support          	AODV_DATA   	AODV_RERR   	AODV_RREP   	AODV_RREQ      5This is the process model for the cct receiver node.         