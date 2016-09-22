/* wlan_txdel_wsn.ps.c */                                                       
/* Modified transmission delay model for IEEE 802.11.	*/
/* It retrieves the transmission data rate from the		*/
/* packet itself instead of the transmitter channel for	*/
/* the computation of the transmission delay. This is	*/
/* needed because the transmissions of all supported	*/
/* WLAN (802.11, 11a, 11b and 11g) data rates use the	*/
/* same tranceiver channel. Hence, the data rate		*/
/* setting of the transmitter channel doesn't always	*/
/* indicate the actual transmission data rate.			*/

/****************************************/
/*        Copyright (c) 1993-2008		*/
/*      by OPNET Technologies, Inc.		*/
/*       (A Delaware Corporation)		*/
/*     7255 Woodmont Av., Suite 250  	*/
/*      Bethesda, MD 20814, U.S.A.      */
/*         All Rights Reserved.			*/
/****************************************/

#include "opnet.h"
#include <oms_tan.h>
#include <oms_rr.h>
#include <oms_pipeline.h> 
#include <string.h> 

static OmsT_Bgutil_Channel_State *
wlan_txdel_chan_state_create (Objid channel_objid);


#if defined (__cplusplus)
extern "C"
#endif
	
//自添加函数
static int
check_rxch (Objid txch, Objid rxch)
{
//var
	Objid 		tx_node_id, rx_node_id;
	Boolean		energy_exhaust;
	
	//double      tx_alt, tx_lat, tx_lon, tx_x, tx_y, tx_z; 
	//double		rx_alt, rx_lat, rx_lon, rx_x, rx_y, rx_z;
	
	double		tx_x, tx_y, rx_x, rx_y;
	double 		distance, distance_threshold;
	char		node_type[10]="type";
//in
	FIN(check_rxch(txch,rxch));
//body
	//printf("在管道阶段 1 阶段。执行接受主寻.\n");
	tx_node_id = op_topo_parent (op_topo_parent (op_topo_parent (txch)));
	rx_node_id = op_topo_parent (op_topo_parent (op_topo_parent (rxch)));
	
	// Now we are ready to perform eligible receiver checks.
	// Station should not hear its own transmission.		
	if (tx_node_id == rx_node_id)
	{
		FRET (OPC_FALSE);
    }
	
	if(op_ima_obj_attr_exists(tx_node_id,"Node Type"))
	{
		op_ima_obj_attr_get(tx_node_id,"Node Type",node_type);
	}
	
	//判断发送sensor节点能量是否已耗尽
	if((strcmp(node_type,"sensor")==0)&&op_ima_obj_attr_exists(tx_node_id,"Energy Exhaust"))
	{
		op_ima_obj_attr_get(tx_node_id,"Energy Exhaust", &energy_exhaust);
		if(energy_exhaust==OPC_TRUE)//能量已耗尽，不可发送数据
		{
			printf("在管道 1 阶段，发射节点能量已耗尽，数据不能发出。\n");
			FRET (OPC_FALSE);
		}
	}
	
	if(op_ima_obj_attr_exists(tx_node_id,"Communication Range"))
	{
		op_ima_obj_attr_get(tx_node_id,"Communication Range",&distance_threshold);
		
		/*
		op_ima_obj_pos_get (tx_node_id, &tx_lat, &tx_lon, &tx_alt, &tx_x, &tx_y, &tx_z);
		op_ima_obj_pos_get (rx_node_id,	&rx_lat, &rx_lon, &rx_alt, &rx_x, &rx_y, &rx_z);
		// Find the distance between the tx_rx pair.
		distance = prg_geo_lat_long_distance_get (tx_lat, tx_lon, tx_alt, rx_lat, rx_lon, rx_alt);
		printf("1、distance=%lf.\n",distance);
		*/
		
		op_ima_obj_attr_get(tx_node_id,"x position",&tx_x); op_ima_obj_attr_get(tx_node_id,"y position",&tx_y);
		op_ima_obj_attr_get(rx_node_id,"x position",&rx_x); op_ima_obj_attr_get(rx_node_id,"y position",&rx_y);
		distance = sqrt((tx_x-rx_x)*(tx_x-rx_x)+(tx_y-rx_y)*(tx_y-rx_y));
		
		//printf("in txdel: distance=%lf.\n",distance);
		if (distance_threshold < distance)
		{
			//printf("in txdel: Distance is too long!\n");
			printf("在管道 1 阶段，距离超过通信范围，数据不能发出。\n");
			FRET (OPC_FALSE);
		}
	}
	
	//判断接收机能量是否耗尽
	if((strcmp(node_type,"sensor")==0)&&op_ima_obj_attr_exists(rx_node_id,"Energy Exhaust"))
	{	
		op_ima_obj_attr_get(rx_node_id,"Energy Exhaust", &energy_exhaust);
		if (energy_exhaust == OPC_TRUE)
		{
			printf("在管道 1 阶段，接收节点能量已耗尽，数据不能发出。\n");
			FRET (OPC_FALSE);
		}
	}
//out
	printf("在管道 1 阶段，数据成功发出.\n");
	FRET(OPC_TRUE);
}
//自添加函数结束
	
void
wlan_txdel_wsn_mt (OP_SIM_CONTEXT_ARG_OPT_COMMA Packet * pkptr)
	{
	OpT_Packet_Size	pklen;
	double			tx_drate, tx_delay;
	Objid	        channel_objid;
	Boolean         is_congestion_area = OPC_FALSE;
	char	        congestion_area_str [3 * OMSC_HNAME_MAX_LEN];
    
	OmsT_Bgutil_Channel_State*		chan_state_ptr = OPC_NIL;
	
//自定义变量
	Objid		tx_ch_objid;
	
	Objid		tx_node_id;
	Boolean 	energy_exhaust;
	double 		energy,E_current_use;
	double		distance, E_elec, E_amp;
	char		file_name[300];
	char		node_type[10]="type";
	FILE 		*in;
	int			node_address;
//自定义变量结束
	
	/** Compute the transmission delay associated with the	**/
	/** transmission of a packet over a radio link.			**/
	FIN_MT (wlan_txdel_wsn (pkptr));

//自添加代码:energy model
	tx_ch_objid = op_td_get_int (pkptr, OPC_TDA_RA_TX_CH_OBJID);
	
	//能量消耗计算并统计
	tx_node_id = op_topo_parent (op_topo_parent (op_topo_parent (tx_ch_objid)));
	
	if(op_ima_obj_attr_exists(tx_node_id,"Node Type"))
	{
		op_ima_obj_attr_get(tx_node_id,"Node Type",node_type);
	}
	
	if((strcmp(node_type,"sensor")==0)&&op_ima_obj_attr_exists(tx_node_id,"Energy Exhaust")&&op_ima_obj_attr_exists(tx_node_id,"Communication Range")&&
		op_ima_obj_attr_exists(tx_node_id,"Energy")&&op_ima_obj_attr_exists(tx_node_id,"E elec")&&
		op_ima_obj_attr_exists(tx_node_id,"E amp")&&op_ima_obj_attr_exists(tx_node_id,"Invalid Time Record File"))
	{
		printf("在管道 1 阶段，是sensor节点，且节点的各 Model 属性名称正确, 开始进行能量消耗计算.\n");
		
		op_ima_obj_attr_get(tx_node_id,"Energy Exhaust", &energy_exhaust);
		if(energy_exhaust==OPC_FALSE)//能量还未尽，计算能量
		{
			op_ima_obj_attr_get(tx_node_id,"Energy",&energy);
			op_ima_obj_attr_get(tx_node_id,"Communication Range",&distance);
			op_ima_obj_attr_get(tx_node_id,"E elec",&E_elec);
			op_ima_obj_attr_get(tx_node_id,"E amp",&E_amp);
			pklen = op_pk_total_size_get (pkptr);
			
			E_current_use = E_elec*pklen + E_amp*pklen*distance*distance;//按照LEACH的能量计算公式计算
			
			energy=energy-E_current_use;
			op_ima_obj_attr_set(tx_node_id,"Energy",energy);
			if(energy<=0)//能量耗尽
			{
				op_ima_obj_attr_set(tx_node_id,"Energy Exhaust", OPC_TRUE);
				
				op_ima_obj_attr_get(tx_node_id,"Invalid Time Record File", file_name);
				op_ima_obj_attr_get(tx_node_id,"MAC Address",&node_address);
				in = fopen(file_name,"at");
				fprintf(in,"%f	%d\r\n",op_sim_time(),node_address);
				fclose(in);
			}
		}
	}else
	{
		if(strcmp(node_type,"sensor")==0)
		{
			printf("在管道 1 阶段, 无法进行能量消耗计算，请检查sensor节点的 Model Attributes 名称在本阶段引用是否正确:\n\
					Energy Exhaust, Communication Range, Energy, E elec, E amp, Invalid Time Record File.\n");
		}
		if(strcmp(node_type,"sink")==0)
		{
			printf("在管道 1 阶段, 当前节点是sink节点，不需要进行能量计算.\n");
		}
	}

	//重新调用管道 0阶段 rsgroup
	op_radio_txch_rxgroup_compute(tx_ch_objid, check_rxch);
	
//自添加代码结束
	
	
	/* Obtain the transmission rate from the packet.		*/
	op_pk_nfd_get_dbl (pkptr, "Tx Data Rate", &tx_drate);

	/* Obtain length of packet. 							*/
	pklen = op_pk_total_size_get (pkptr);

	/* Compute time required to complete transmission of	*/
	/* packet. 												*/
	tx_delay = pklen / tx_drate;

	/* Place transmission delay result in packet's 			*/
	/* reserved transmission data attribute. 				*/
	op_td_set_dbl (pkptr, OPC_TDA_RA_TX_DELAY, tx_delay);
	
	/* Also update the transmitter's data rate TDA with the	*/
	/* information retrieved from the packet to be used in	*/
	/* the following pipeline stages.						*/
	op_td_set_dbl (pkptr, OPC_TDA_RA_TX_DRATE, tx_drate);
	
	/* Check whether congestion area monitoring is enabled. */
	if (CONGESTION_AREAS_ENABLED)
		{
   	    int       num_flows = 0;
	    double    utilization = 0.0;

		/* Obtain object id of transmitter channel forwarding transmission. */
		channel_objid = op_td_get_int (pkptr, OPC_TDA_RA_TX_CH_OBJID);
		
		/* Create a channel state to store the state information.           */
		if ((chan_state_ptr = (OmsT_Bgutil_Channel_State *) op_ima_obj_state_get (channel_objid)) == OPC_NIL)
			{
			chan_state_ptr = wlan_txdel_chan_state_create (channel_objid);
			}
	
		/* If this is a tracer packet update the congestion area info. */ 
		if (op_pk_encap_flag_is_set (pkptr, OMSC_BGUTIL_ENCAP_FLAG_INDEX))
			{	
			is_congestion_area = oms_bgutil_state_info_update (pkptr, tx_drate, 
				chan_state_ptr->port_name_ptr,	&chan_state_ptr->congestion_area,
				&num_flows, &utilization, &chan_state_ptr->routed_bgutil_state_ptr);
			
			if (is_congestion_area)
				{
				/* Add congestion info into the route record string. */
				sprintf (congestion_area_str, "%s(CONGESTION <util=%.2f num_flows=%d>),None", 
					chan_state_ptr->node_name_ptr, 100.0 * utilization, num_flows);
				}
			}
		}
		
	/* Dump routes for only packets	that have requested for	*/
	/* route information by setting the correct flag.		*/
	if (op_pk_encap_flag_is_set (pkptr, OMSC_RR_ENCAP_FLAG_INDEX))
		{	
        /* Obtain the channel state if NIL. */
	    if (chan_state_ptr == OPC_NIL)	
			{
        	/* Obtain object id of transmitter channel forwarding transmission. */
			channel_objid = op_td_get_int (pkptr, OPC_TDA_RA_TX_CH_OBJID);
		
	        /* Get the channel state.  */
			if ((chan_state_ptr = (OmsT_Bgutil_Channel_State*) op_ima_obj_state_get (channel_objid)) == OPC_NIL)
				{
				/* Create a new channel state.  */
				chan_state_ptr = wlan_txdel_chan_state_create (channel_objid);
				}
			}
		
        /* Update information about traversed node/links if	*/
		/* this is a bgutil tracer packet.					*/
        if (is_congestion_area)
			{
			/* Include the congestion infromation. */
  	 		oms_rr_info_update (pkptr, congestion_area_str);
			}
		else
			{
			oms_rr_info_update (pkptr, chan_state_ptr->node_link_name_ptr);
			}
		}
	
	FOUT
	}


static OmsT_Bgutil_Channel_State *
wlan_txdel_chan_state_create (Objid channel_objid)
	{	
    Objid     port_objid;  
	char      tmp_name        [OMSC_HNAME_MAX_LEN]     = "\0";
    char	  node_link_name  [2 * OMSC_HNAME_MAX_LEN] = "\0";

    OmsT_Bgutil_Channel_State*		chan_state_ptr = OPC_NIL;

	/* Creates and populates the tx channel state info. */
	FIN (wlan_txdel_chan_state_create (Objid channel_objid));

    /* Channel state information doesn't exist. Lock	*/
	/* the global mutex before continuing.				*/
	op_prg_mt_global_lock ();

	/* Check again since another thread may have		*/
	/* already set up the state information.			*/
	if ((chan_state_ptr = (OmsT_Bgutil_Channel_State*) op_ima_obj_state_get (channel_objid)) == OPC_NIL)
		{
		/* Obtain a new channel state structure. This     */
		/* structure will be associated with the channel. */
		chan_state_ptr = oms_bgutil_channel_state_create ();
		
        /* Determine the port objid. */ 
		port_objid = op_topo_parent (op_topo_parent (channel_objid));
		
		/* Determine the name of the sending port and store	*/
		/* it for future use. 								*/
		oms_tan_hname_get (port_objid, tmp_name);		
	
		/* Allocate memory and store the port name. */
		chan_state_ptr->port_name_ptr = (char *) op_prg_mem_alloc 
			(sizeof (char) * (strlen (tmp_name) + 1));
		strcpy (chan_state_ptr->port_name_ptr, tmp_name); 

        /* Obtain the node name. */
		oms_tan_hname_get (op_topo_parent (port_objid), tmp_name);
		
       	/* Allocate memory and store the node name. */
		chan_state_ptr->node_name_ptr = (char *) op_prg_mem_alloc 
			(sizeof (char) * (strlen (tmp_name) + 1));
		strcpy (chan_state_ptr->node_name_ptr, tmp_name); 
		
		/* Create the node_link_str string. */
		strcpy (node_link_name, tmp_name);
		strcat (node_link_name, ",None");
  
		/* Allocate memory and store the node_link_name_str. */
		chan_state_ptr->node_link_name_ptr = (char *) op_prg_mem_alloc 
			(sizeof (char) * (strlen (node_link_name) + 1));
		strcpy (chan_state_ptr->node_link_name_ptr, node_link_name); 
		
		/* Associate the state with the channel; the next call to op_ima_obj_state_get() for 	*/
		/* this channel will now return the address of this channel state. 						*/
		op_ima_obj_state_set (channel_objid, chan_state_ptr);	
		}

	/* Unlock the global mutex.							*/
	op_prg_mt_global_unlock ();

	/* Return the channel state */
	FRET (chan_state_ptr);
	}
