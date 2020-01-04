#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#define DISCONNECTED
#define R 6371000 // meters
#define DEG_TO_RAD 2*M_PI/360 

//constant used to read file
const int no_lines=25313047;
const int no_nodes=23895681;
const int no_ways=1417363;
const int id_node_start=240949599;
const int id_node_goal=195977239;

/////////////////// DEFINE STRUCTURES ////////////////

//### Format: node|@id|@name|@place|@highway|@route|@ref|@oneway|@maxspeed|node_lat|node_lon
typedef struct 
{
	unsigned long id;	//node ID ($1)
	double lat;	// coordinate position: latitude ($9)
	double lon;	// coordinate position: longitude ($10)
	char *name;	//name ($2)
	unsigned short nsucc; // Number of node successors; i. e. length of successors
	//unsigned long *successors; --- Instead of using the successors list in the structure we are going to create a global matrix to store these values
	//int oneway_node;	--- one-way ($7), 'boolean var' to check if needs to be read in reverse, takes value 1 for True, 0 for False
} node;
//___________________________________________________

//typedef char Queue;
//enum whichQueue {NONE, OPEN, CLOSED};

typedef struct 
{
	double g, h;
	unsigned long parent;
	//Queue whq;
	int queue; //Takes values NONE=0, OPEN=1, CLOSED=2 (default 0)
	int next;
} AStarStatus;

/////////////////////////////////////////////////////


/////////////////// DEFINE GLOBAL VARIABLES /////////////
//Define as global variables
//Create the vector of node structures where the information for each node is stored
node nodes[no_nodes]; 
//Create a matrix to store the list of succesors of each node (max. num. of succ = 16) 
//---This is instead of using the successors list in the node structure
int succ_matrix[16][no_nodes];
//Create the vector of AStarStatus structures where the information on the status of each node is stored
AStarStatus node_status[no_nodes];
//////////////////////////////////////////////////////


///////////////////////// FUNCTIONS ///////////////////////

int binary_search(unsigned long ix)
{
	int first = 0;
	int last = no_nodes-1;
	int middle = last/2;

	while( first <= last)
	{
		if      (nodes[middle].id == ix)	return middle;
		else if (nodes[last].id == ix)		return last;
		else if ((last-first)==1)			return -1;
		else if (nodes[middle].id < ix)		first = middle;
		else if (nodes[middle].id > ix)		last = middle;
		else
			{
				printf("Error: No condition is satisfied in binary search.\n");
    			exit(1);
    		}

		middle = (first+last)/2;
	}

	return -1;
}
//________________________________________________________

void fill_succ(int index_a, int index_b, bool is_oneway)
{
	succ_matrix[nodes[index_a].nsucc][index_a]=index_b;
	nodes[index_a].nsucc++;
	if (!is_oneway)
	{
		succ_matrix[nodes[index_b].nsucc][index_b]=index_a;
		nodes[index_b].nsucc++;
	}
	//printf("Index_a:%i, Nsucc_a:%u, Index_b:%i\n", index_a, nodes[index_a].nsucc-1, succ_matrix[nodes[index_a].nsucc-1][index_a]);
}
//________________________________________________________

double compute_dist(double lat1, double lon1, double lat2, double lon2)
{	
	double phi1, phi2, delta_phi, delta_lambda, a, c;

	phi1 = lat1*DEG_TO_RAD;
	phi2 = lat2*DEG_TO_RAD;
	delta_phi = (lat2-lat1)*DEG_TO_RAD;
	delta_lambda = (lon2-lon1)*DEG_TO_RAD;

	a = sin(delta_phi*0.5) * sin(delta_phi*0.5) +
        cos(phi1) * cos(phi2) *
        sin(delta_lambda*0.5) * sin(delta_lambda*0.5);
	c = 2 * atan2(sqrt(a), sqrt(1-a));

	return R*c;
}
//________________________________________________________

int insert_node(int node_top, int node_new)
{
	double f_curr, f_new;
	int curr_node, next_node, prev_node;
	bool is_last = false;
	int list_elem = 0;

	curr_node = node_top;
	f_new = node_status[node_new].h + node_status[node_new].g;

	//printf("Inserting node %i into the open list\n", node_new);

	while(!is_last)
	{
		f_curr = node_status[curr_node].h + node_status[curr_node].g;
		
		if (f_new < f_curr)
		{
			node_status[node_new].next = curr_node;
			if (list_elem > 0) 
			{
				node_status[prev_node].next = node_new;
				break;
			}
			else
				return node_new;
		}
		else
		{
			next_node = node_status[curr_node].next;
			if (next_node == -1) 
			{
				node_status[curr_node].next = node_new;
				node_status[node_new].next = -1;
				is_last = true;
			}
			else
			{
				prev_node = curr_node;
				curr_node = next_node;
			}
		}
		list_elem++;
	}
	return node_top;
}
//________________________________________________________

int delete_node(int node_kill, int node_top)
{
	int curr_node, next_node;
	curr_node = node_top;

	if (node_top == node_kill)
	{
		node_top = node_status[node_top].next;
		return node_top;
	}

	while(node_status[curr_node].next > -1)
	{
		next_node = node_status[curr_node].next;
		if (next_node == node_kill)
		{
			node_status[curr_node].next = node_status[next_node].next;
			node_status[next_node].next = -1;
			break;
		}
		curr_node = next_node;
	}

	if (curr_node == -1)
	{
		printf("Error: The node to kill is not in the open list\n");
		exit(1);
	}
	
	return node_top;
}
//________________________________________________________

void print_open_list(int top_node)
{
	int curr_node;

	printf("-------------------------------------------------\n");
	printf("OPEN LIST:\n");
	printf("%i\n", top_node);
	curr_node = top_node;

	while(node_status[curr_node].next > -1)
	{
		printf("%i\n", node_status[curr_node].next);
		curr_node = node_status[curr_node].next;
	}

	printf("-------------------------------------------------\n");
}
//________________________________________________________

void print_parent_list(int last_node)
{
	int curr_node;

	printf("-------------------------------------------------\n");
	printf("FINAL LIST:\n");
	printf("Node id: %lu \t | Distance: %.2f \t | Name: %s\n", nodes[last_node].id, node_status[last_node].g, nodes[last_node].name);
	curr_node = last_node;

	while(node_status[curr_node].parent != -1)
	{
		printf("Node id: %lu \t | Distance: %.2f \t | Name: %s\n", nodes[node_status[curr_node].parent].id, node_status[node_status[curr_node].parent].g, nodes[node_status[curr_node].parent].name);
		curr_node = node_status[curr_node].parent;
	}

	printf("-------------------------------------------------\n");
}
///////////////////////////////////////////////////



///////////////////MAIN FUNCTION////////////////////////

int main(int argc, char *argv[]){

	clock_t t_begin = clock();

	//__________________LOAD FILE_________________________

	//First, we need to load the file from the arguments in the command line
   
    char *path=argv[1];

	printf("%s\n", path);
	printf("The algorithm will start shortly...\n");

	//We open the file and load its content to fp (*fp is the pointer of this)
	//Raise an error if this is not succesfull

    FILE *fp = fopen(path,"r");
    if(fp==NULL)
    {
    	perror("Unable to open file :(\n");
    	exit(1);
    }
	//___________________________________________________    


	//Next we need to read the file line by line, for this we use the getline() function
	//Loop to go over lines, buffer is pointer of string (line) that is read
	//&buffer goes to the memory address (first entry) of line to begin reading 
	//Loop starts at -3 to skip header and maintain coherence
	
	for(int j = -3; j < no_lines-3; j++)
	{
		//__________________READ LINES_______________________

		size_t  length_line = 0;
		char *buffer = NULL;

		getline(&buffer,&length_line,fp);
		//___________________________________________________ 

		//Next we use strsep() to divide the line by fields according to a token "|"
		//Loop to go over fields, strsep() sequentially cuts the initial string, need a copy of buffer
		//Field is temporarily stored in 'token' and then transferred to the node structure
		//We use the 'field' counter to identify the field number

		char *copy = buffer;
		char *token = NULL;
		int field = 0;

		//Define variables
		
		bool is_node = false;
		bool is_way = false;
		bool is_oneway = false;
		int index_a=-1, index_b=-1;


		//Skip first three lines of the file that form the header
		if (j < 0)
			continue;


		//__________________PROCESS LINES_______________________
		do
		{
			token=strsep(&copy,"|");
			
			//The ($0) field determines if the line is a node, a way or a relation
			//We use booleans that become true when the string comparison is fulfilled

			if(field==0 && !strncmp(token,"n",1))
				is_node = true;

			if (is_node) //Enters when is_node==true
			{
				if(field==1)
					//Use strtoul() to convert string to unsigned long
					//Fill the ID structure's member variable for the corresponding node
					nodes[j].id = strtoul(token,NULL,10);	
				
				if(field==2)
					//Fill the 'name' structure member variable for the corresponding node
					nodes[j].name = token;

				if(field==9) 
					//Use atof() to convert string to double
					//Fill the latitude and longitude structure member variables for the corresponding node
					nodes[j].lat = atof(token);	

				if(field==10)
					nodes[j].lon= atof(token);
			} 

			if(field==0 && !strncmp(token,"w",1))
				is_way = true;

			if (is_way && token) //Enters when is_way==true and when token!=NULL
			{
				if(field==7 && !strncmp(token,"o",1))
					//Check if the way is one way or two ways
					is_oneway = true;

				//All nodes in the way's list of nodes need to be processed in order to fill the successors matrix
				if(field>=9)
				{
					if (index_a<0) 
						index_a = binary_search(strtoul(token,NULL,10));
					else
					{
						index_b = binary_search(strtoul(token,NULL,10));

						if (index_b>=0)
						{
							fill_succ(index_a, index_b, is_oneway);
							index_a = index_b;
						}
						else
						{
							//To control how we want to deal with unexisting nodes in the way's node list 
							#ifdef DISCONNECTED //If DISCONNECTED is defined as a global variable then the node pairing is restarted
								index_a = -1;
							#else //If DISCONNECTED is not defined the previous and the following nodes are connected between them
								;
							#endif
						}
					}
				}
			}
			
			field++;

		} while(token);

		//_____________________________________________________


		//__________________SILLY PRINTS_______________________

		if(j<=no_nodes)
		{
			if(j!=(no_nodes-1) && j%1000000==0)
			{
				printf("\r\033[1;32mProcessing nodes...\033[0m %.2f%%", (1.*j/(no_nodes-1))*100);
				fflush(stdout);
			}
			else if(j==(no_nodes-1))
				printf("\r\033[1;32mProcessing nodes...\033[0m 100%%    \n");
		}
		else if(j<(no_nodes+no_ways))
		{
			if(j!=(no_nodes+no_ways-1) && j%100000==0)
			{
				printf("\r\033[1;31mProcessing ways... \033[0m %.2f%%", (1.*(j-no_nodes)/(no_ways-1))*100);
				fflush(stdout);
			}
			else if(j==(no_nodes+no_ways-1))
				printf("\r\033[1;31mProcessing ways...\033[0m 100%%    \n");
		}
		else
			break;
		//___________________________________________________ 
	}

	clock_t t_end = clock();
	double delta_t = (double)(t_end-t_begin) / CLOCKS_PER_SEC;

	//-----------------------------------------------------
	//                       A STAR                        
	//-----------------------------------------------------

	clock_t t_A_star_begin = clock();

	int node_start, node_goal, node_current, node_successor, top_node;
	int open_count=0;
	double dist_time=0, insert_time=0, cond_time=0;
	int point_count=0;
	//int iter_count=0;
	double successor_current_cost;

	node_start = binary_search(id_node_start);
	node_goal = binary_search(id_node_goal);

	//______________INITIALIZATION OF START NODE_____________

	node_status[node_start].queue = 1;
	open_count++;
	node_status[node_start].parent = -1;
	node_status[node_start].next = -1;
	node_status[node_start].h = compute_dist(nodes[node_start].lat, nodes[node_start].lon, nodes[node_goal].lat, nodes[node_goal].lon);
	node_status[node_start].g = 0;
	node_current = node_start;
	printf("Initializing node start: %i\n", node_start);
	//print_open_list(node_start);
	//_______________________________________________________

	//_______________________MAIN LOOP_______________________

	while(open_count)
	{
		if (node_current == node_goal)
			break;

		// printf("_______________________________________________\n");
		// printf("VISITING NODE %i (With successors:", node_current);
		// for (int i = 0; i < nodes[node_current].nsucc; i++)
		// 	printf(" %i", succ_matrix[i][node_current]);
		// printf(")\n");
		// printf("_______________________________________________\n");

		
		for(int k = 0; k < nodes[node_current].nsucc; k++)
		{
			clock_t t_dist_begin = clock();
			node_successor = succ_matrix[k][node_current];
			clock_t t_dist_end = clock();
			successor_current_cost = node_status[node_current].g + compute_dist(nodes[node_current].lat, nodes[node_current].lon, nodes[node_successor].lat, nodes[node_successor].lon);
			
			double delta_t_dist = (double)(t_dist_end-t_dist_begin) / CLOCKS_PER_SEC;
			dist_time = dist_time + delta_t_dist;

			// clock_t t_cond_begin = clock();
			if (node_status[node_successor].queue == 1)
			{
				if (node_status[node_successor].g <= successor_current_cost)
					continue;

				top_node = delete_node(node_successor, node_current);
				//printf("Revisiting node %i, may change its position whithin the open list\n", node_successor);

				if (top_node != node_current)
				{
					printf("Error: Coherence error, node successor should be behind node current in open list\n");
					exit(1);
				}
			}
			else if (node_status[node_successor].queue == 2)
			{
				if (node_status[node_successor].g <= successor_current_cost)
					continue;
				
				node_status[node_successor].queue = 1;
				open_count++;
			}
			else
			{
				node_status[node_successor].queue = 1;
				open_count++;
				node_status[node_successor].h = compute_dist(nodes[node_start].lat, nodes[node_start].lon, nodes[node_goal].lat, nodes[node_goal].lon);
			}
			// clock_t t_cond_end = clock();
			// double delta_t_cond = (double)(t_cond_end-t_cond_begin) / CLOCKS_PER_SEC;
			// cond_time = cond_time + delta_t_cond;

			node_status[node_successor].g = successor_current_cost;
			node_status[node_successor].parent = node_current;

			// clock_t t_insert_begin = clock();
			top_node = insert_node(node_current, node_successor);
			// clock_t t_insert_end = clock();
			// double delta_t_insert = (double)(t_insert_end-t_insert_begin) / CLOCKS_PER_SEC;
			// insert_time = insert_time + delta_t_insert;
			//print_open_list(top_node);

			point_count++;
		}

		node_status[node_current].queue = 2;
		open_count--;
		top_node = delete_node(node_current, top_node);
		//printf("Moving node %i to the closed list\n", node_current);
		//print_open_list(top_node);
		node_current = top_node;

		// if (iter_count>10)
		// 	break;
		// iter_count++;
	}

	//print_parent_list(node_goal);
	//________________________________________________________


	clock_t t_A_star_end = clock();
	double delta_t_A_star = (double)(t_A_star_end-t_A_star_begin) / CLOCKS_PER_SEC;
	printf("Reading and building structures finished in %.2f seconds\n", delta_t);
	printf("Compute distance funtion finished in average %.8f seconds, %.2f %i\n", dist_time/point_count, dist_time, point_count);
	printf("Condition finished in average %.8f seconds, %.2f %i\n", cond_time/point_count, cond_time, point_count);
	printf("Insert function finished in average %.8f seconds, %.2f %i\n", insert_time/point_count, insert_time, point_count);
	printf("A star finished in %.2f seconds\n", delta_t_A_star);

	return 0;
}

////////////////////////////////////////////////////