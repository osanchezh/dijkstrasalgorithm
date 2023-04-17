// C program for Dijkstra's single source shortest path
// algorithm. The program is for adjacency matrix
// representation of the graph

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

// Number of vertices in the graph
#define V 9

// A utility function to find the vertex with minimum
// distance value, from the set of vertices not yet included
// in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
	// Initialize min value
	int min = INT_MAX, min_index;

	for (int v = 0; v < V; v++){
		if (sptSet[v] == false && dist[v] <= min){
			min = dist[v];
			min_index = v;
			//printf("min= %d, min_index= %d \n", min, min_index);
	         }
	}
	return min_index;
}



int buscar(int nodos[], int buscar, int arreglo1[], int arregloSumador[], int repeticion){
  int inicio=0;
  for(int y=0;y<V;y++){
   if(arreglo1[y]>=0){
    inicio=y;
   }
  } 
  if(inicio>0){
    inicio++;
  }
  if(inicio==0 && repeticion){
    arreglo1[0] = buscar;
    arregloSumador[0]=0;
    inicio++;
  }

  //printf("buscar=%d\n",buscar);
  
  int conGlo=inicio;
  arreglo1[conGlo] = nodos[buscar];
  //printf(">>>buscar.-A = %d - %d \n", nodos[buscar],arreglo1[conGlo]);
  conGlo++;
 
  for(int t=nodos[buscar]; t>0; t--){
     //printf(">>t:%d \n",t);
     //printf(" (%d) .- buscarA. = %d - %d, ar= %d \n",t, nodos[t],t,arreglo1[conGlo-1]);
     if(t==arreglo1[conGlo-1]){

       arreglo1[conGlo]=nodos[t];
       //printf(" >>>>(%d) .- buscarB. = %d - %d , ar= %d \n",t, nodos[t],t,arreglo1[conGlo-1]);
       conGlo++;
    // desfuncional   //arreglo1[conGlo]=nodos[t];
    // desfuncional //printf(" >>>>(%d) .- buscarC. = %d - %d , ar= %d \n",t, nodos[t],t,arreglo1[conGlo]);
      
    
   }
     
     //printf(" (%d) .- buscarD. = %d - %d , ar= %d \n",t, nodos[t],t,arreglo1[conGlo]);
    

  }
  int last=0;
     for(int g=0 ; g<V;g++)
     {
       if(arreglo1[g]>=0){
         last=arreglo1[g];
       }
      // printf("secuencia= %d ;\n",arreglo1[g]);
     }
  return last;
}




void findRoots(int parent[], int graph[V][V]){
    int nodos[V];
    nodos[0]=0;
    for (int i = 1; i < V; i++){
        nodos[i]=parent[i];
    }
    
    int grafo[V][V];
    
    for(int q=0;q < V;q++){
     for(int w=0;w < V;w++){
      grafo[q][w] = -1;
     }
    }
    
    
    for(int h=0;h<V;h++)
    { 
	int arreglo1[V];
	int arregloSumador[V];
	int costo[V];
     	for(int g=0 ; g<=V;g++)
     	{
       		arreglo1[g]=-1;
       		arregloSumador[g]=0;
       		costo[g]=-1;
     	}
        int valor = buscar(nodos,h, arreglo1,arregloSumador, true);
        while(valor!=0||valor<0) {
         //printf("valor= %d \n", valor);
         valor= buscar(nodos,valor, arreglo1,arregloSumador, false);
         //printf("valor= %d \n", valor);
        }
        for(int k=0;k<V;k++){
           grafo[h][k]=arreglo1[k];
        }
     }
     
    int costos[V][V];
    int costosTotal[V][V];
    for(int h=0;h<V;h++){
    	for(int k=0;k<V;k++){
    		costos[h][k]=-1;
    		costosTotal[h][k]=-1;
    	}
    }
    for(int h=0;h<V;h++)
    { 
     int costoTotal=0;
     for(int k=0;k<V;k++){
        if(grafo[h][k]>=0){
         int igrafo=grafo[h][k];
         int costo=0;
         if(igrafo!=0){
            costo=graph[igrafo][parent[igrafo]];
         }
         costoTotal=costoTotal+costo;
         printf("grafo[%d][%d]=%d , costo: %d , costoTotal: %d\n",h,k,igrafo,costo,costoTotal);
         costos[h][k]=costo;
         costosTotal[h][k]=costoTotal;
         
        }
     }
    }
}

void printMST(int parent[], int graph[V][V])
{
    printf("Edge \tWeight\n");
    int nodos[V];
    nodos[0]=0;
    for (int i = 1; i < V; i++){
        printf("%d - %d \t%d \n", parent[i], i,
               graph[i][parent[i]]);
        nodos[i]=parent[i];
    }
        
}


// A utility function to print the constructed distance
// array
void printSolution(int dist[])
{
	printf("Vertex \t\t Distance from Source\n");
	for (int i = 0; i < V; i++)
		printf("%d \t\t\t\t %d\n", i, dist[i]);
}

// Function that implements Dijkstra's single source
// shortest path algorithm for a graph represented using
// adjacency matrix representation
void dijkstra(int graph[V][V], int src)
{
	 int parent[V];
	 
	int dist[V]; // The output array. dist[i] will hold the
				// shortest
	// distance from src to i

	bool sptSet[V]; // sptSet[i] will be true if vertex i is
					// included in shortest
	// path tree or shortest distance from src to i is
	// finalized

	// Initialize all distances as INFINITE and stpSet[] as
	// false
	for (int i = 0; i < V; i++) {
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}

	// Distance of source vertex from itself is always 0
	//printf("limite: %d \n\n",INT_MAX);
	dist[src] = 0;
	parent[0] = -1;
	// Find shortest path for all vertices
	for (int count = 0; count < V -1; count++) {
		//printf("------------%d--------------\n",count);
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to
		// src in the first iteration.
		int u = minDistance(dist, sptSet);
		//printf("distancia: %d ,\n",u);
		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < V; v++) {

			// Update dist[v] only if is not in sptSet,
			// there is an edge from u to v, and total
			// weight of path from src to v through u is
			// smaller than current value of dist[v]
			if (!sptSet[v] && graph[u][v]
				&& dist[u] != INT_MAX
				&& dist[u] + graph[u][v] < dist[v]){
				//printf("dist[u]>>>%d , %d , %d , %d , %d , sptSet %d \n", dist[u], graph[u][v], dist[v], v, u, sptSet[v]);
				dist[v] = dist[u] + graph[u][v];
				//printf("dist[v]>>>%d , %d , %d , %d , %d , sptSet %d \n", dist[v],  dist[u], graph[u][v], v, u, sptSet[v]);
				
				parent[v] = u;
				//printf(">>>%d ->  %d", parent[v], u);
				
				}
		}
		//printf("------------%d--------------\n",count);
	}

	// print the constructed distance array
	printSolution(dist);
	
	printMST(parent, graph);
	findRoots(parent, graph);
	
}

// driver's code
int main()
{
	/* Let us create the example graph discussed above */
	int graph[V][V] = { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
						{ 4, 0, 8, 0, 0, 0, 0, 11, 0 },
						{ 0, 8, 0, 7, 0, 4, 0, 0, 2 },
						{ 0, 0, 7, 0, 9, 14, 0, 0, 0 },
						{ 0, 0, 0, 9, 0, 10, 0, 0, 0 },
						{ 0, 0, 4, 14, 10, 0, 2, 0, 0 },
						{ 0, 0, 0, 0, 0, 2, 0, 1, 6 },
						{ 8, 11, 0, 0, 0, 0, 1, 0, 7 },
						{ 0, 0, 2, 0, 0, 0, 6, 7, 0 } };

	// Function call
	dijkstra(graph, 0);

	return 0;
}

