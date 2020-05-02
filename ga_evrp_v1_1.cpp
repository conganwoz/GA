#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <chrono>
#include <iostream>

//using namespace std;
using namespace std::chrono;
using namespace std;

// -- data problem --
int DIMENTION = 0;
int NUM_STATIONS = 0;
int NUM_CUSTOMERS = 0;
int NUM_VEHICLES = 0;

double MAX_CAPACITY_VH = 0.0;
double MAX_ENERGY_VH = 0.0;
double ENG_CONSUMTION = 0.0;
double OPTIMAL_VALUE = 0.0;

double **Distances;
double Demands[2000];
double **Coords;

int **Best_Stations;
double **Best_Station_Distances;

double **F_CAP;
double **B_CAP;
double **F_ENERGY;
double **B_ENERGY;

// -- data in GA process --
int **Routes;
int **Route_In_Pool;
bool **Through_Stations;
bool **Through_Stations_In_Pool;
double *Fitnesses;
double Alpha, Beta, H;
double Roulette_Wheel_Arr[1000];
int Parent_Pool[700];
bool Is_In_Pool[1000];
bool Deprecate_In_Next_Gen[1000];

int *Temp_Sequence_Customers;

// ---- meta data ----
double Costs[1000];
double Over_Capacities[1000];
double Over_Energies[1000];
double Fitness[1000];
bool Is_Feasible[1000];

void create_space_mem()
{
  int i = 0;
  Coords = (double **)malloc(DIMENTION * sizeof(double *));
  Distances = (double **)malloc(DIMENTION * sizeof(double *));
  Best_Stations = (int **)malloc(NUM_CUSTOMERS * sizeof(int *));
  Best_Station_Distances = (double **)malloc(NUM_CUSTOMERS * sizeof(int *));
  Temp_Sequence_Customers = (int *)malloc(NUM_CUSTOMERS * sizeof(int));

  F_CAP = (double **)malloc(1000 * sizeof(double *));
  B_CAP = (double **)malloc(1000 * sizeof(double *));
  F_ENERGY = (double **)malloc(1000 * sizeof(double *));
  B_ENERGY = (double **)malloc(1000 * sizeof(double *));

  Routes = (int **)malloc(1000 * sizeof(int *));
  Route_In_Pool = (int **)malloc(700 * sizeof(int *));
  Through_Stations = (bool **)malloc(1000 * sizeof(bool *));
  Through_Stations_In_Pool = (bool **)malloc(700 * sizeof(bool *));

  for (i = 0; i < DIMENTION; i++)
  {
    Coords[i] = (double *)malloc(2 * sizeof(double));
    Distances[i] = (double *)malloc(DIMENTION * sizeof(double));
    if (i < NUM_CUSTOMERS)
    {
      Best_Stations[i] = (int *)malloc(NUM_CUSTOMERS * sizeof(int));
      Best_Station_Distances[i] = (double *)malloc(NUM_CUSTOMERS * sizeof(double));
    }
  }

  for (i = 0; i < 1000; i++)
  {
    F_CAP[i] = (double *)malloc(NUM_CUSTOMERS * sizeof(double));
    B_CAP[i] = (double *)malloc(NUM_CUSTOMERS * sizeof(double));
    F_ENERGY[i] = (double *)malloc(NUM_CUSTOMERS * sizeof(double));
    B_ENERGY[i] = (double *)malloc(NUM_CUSTOMERS * sizeof(double));
    Routes[i] = (int *)malloc((NUM_CUSTOMERS + NUM_VEHICLES + 1) * sizeof(int));
    Through_Stations[i] = (bool *)malloc((NUM_CUSTOMERS + NUM_VEHICLES) * sizeof(bool));
    Through_Stations_In_Pool[i] = (bool *)malloc((NUM_CUSTOMERS + NUM_VEHICLES) * sizeof(bool));
    if (i < 700)
      Route_In_Pool[i] = (int *)malloc((NUM_CUSTOMERS + NUM_VEHICLES + 1) * sizeof(int));
  }
}

void read_file(char *file_src)
{
  int i;
  FILE *infile;
  infile = fopen(file_src, "r");
  if (infile == NULL)
  {
    printf("READ FILE ERROR\n");
    exit(-1);
  }
  else
  {
    printf("READ FILE SUCCESS\n");
  }

  fscanf(infile, "%lf\n", &OPTIMAL_VALUE);
  fscanf(infile, "%d\n", &NUM_VEHICLES);
  fscanf(infile, "%d\n", &DIMENTION);
  fscanf(infile, "%d\n", &NUM_STATIONS);
  fscanf(infile, "%lf\n", &MAX_CAPACITY_VH);
  fscanf(infile, "%lf\n", &MAX_ENERGY_VH);
  fscanf(infile, "%lf\n", &ENG_CONSUMTION);

  NUM_CUSTOMERS = DIMENTION;
  DIMENTION = NUM_CUSTOMERS + NUM_STATIONS;

  printf("data: dimention: %d - num_customer: %d - capacity_vh: %lf - energy_vh: %lf - energy_consumtion: %lf - num_vehicles: %d - num_station: %d\n", DIMENTION, NUM_CUSTOMERS, MAX_CAPACITY_VH, MAX_ENERGY_VH, ENG_CONSUMTION, NUM_VEHICLES, NUM_STATIONS);
  create_space_mem();

  double index, x, y;
  double demand = 0.0;
  for (i = 0; i < NUM_CUSTOMERS; i++)
  {
    fscanf(infile, "%lf %lf %lf\n", &index, &x, &y);
    Coords[i][0] = x;
    Coords[i][1] = y;
  }

  for (i = NUM_CUSTOMERS; i < DIMENTION; i++)
  {
    fscanf(infile, "%lf %lf %lf\n", &index, &x, &y);
    Coords[i][0] = x;
    Coords[i][1] = y;
  }

  for (i = 0; i < NUM_CUSTOMERS; i++)
  {
    fscanf(infile, "%lf %lf\n", &index, &demand);
    Demands[i] = demand;
  }

  fclose(infile);
  printf("\n");
}

void find_best_stations(int i, int j)
{
  double min_distance = 100000000;
  int k;
  int best_stat = -1;
  for (k = NUM_CUSTOMERS; k < DIMENTION; k++)
  {
    if (Distances[i][k] + Distances[j][k] < min_distance)
    {
      min_distance = Distances[i][k] + Distances[j][k];
      best_stat = k;
    }
  }

  Best_Station_Distances[i][j] = min_distance;
  Best_Station_Distances[j][i] = min_distance;

  Best_Stations[i][j] = best_stat;
  Best_Stations[j][i] = best_stat;
}

void prepare_data()
{
  int i, j, k = 0;
  for (i = 0; i < DIMENTION; i++)
  {
    for (j = i + 1; j < DIMENTION; j++)
    {
      double distance = sqrt((Coords[i][0] - Coords[j][0]) * (Coords[i][0] - Coords[j][0]) + (Coords[i][1] - Coords[j][1]) * (Coords[i][1] - Coords[j][1]));
      Distances[i][j] = distance;
      Distances[j][i] = distance;
    }
  }

  for (i = 0; i < NUM_CUSTOMERS; i++)
  {
    for (j = i + 1; j < NUM_CUSTOMERS; j++)
    {
      find_best_stations(i, j);
    }
  }
}

double dist_consum(double distance)
{
  return distance * ENG_CONSUMTION;
}

void init_population()
{
  int i, j, k, rand_through;
  srand((unsigned)time(NULL));
  int num_customer_each_route = (int)((NUM_CUSTOMERS - 1) / NUM_VEHICLES) + 2;
  for (i = 0; i < 1000; i++)
  {
    int rand_j = (int)(rand() % (NUM_CUSTOMERS - 1)) + 1;
    k = 1;
    int route_num = NUM_CUSTOMERS - 1;
    for (j = rand_j; j < NUM_CUSTOMERS; j++)
    {
      if (k % num_customer_each_route == 0)
      {
        route_num++;
        Routes[i][k] = route_num;
        rand_through = (int)(rand() % 2);
        if (rand_through == 0)
          Through_Stations[i][k] = false;
        else
          Through_Stations[i][k] = true;

        k++;
      }
      Routes[i][k] = j;
      rand_through = (int)(rand() % 2);
      if (rand_through == 0)
        Through_Stations[i][k] = false;
      else
        Through_Stations[i][k] = true;

      k++;
    }
    for (j = 1; j < rand_j; j++)
    {
      if (k % num_customer_each_route == 0)
      {
        route_num++;
        Routes[i][k] = route_num;
        rand_through = (int)(rand() % 2);
        if (rand_through == 0)
          Through_Stations[i][k] = false;
        else
          Through_Stations[i][k] = true;
        k++;
      }
      Routes[i][k] = j;
      rand_through = (int)(rand() % 2);
      if (rand_through == 0)
        Through_Stations[i][j] = false;
      else
        Through_Stations[i][j] = true;
      k++;
    }
    route_num++;
    Routes[i][k] = route_num;

    // set Throught station for depot
    rand_through = (int)(rand() % 2);
    if (rand_through == 0)
      Through_Stations[i][0] = false;
    else
      Through_Stations[i][0] = true;

    Routes[i][0] = 0;

    // rand_through = (int)(rand() % 2);
    // if (rand_through == 0)
    //   Through_Stations[i][NUM_CUSTOMERS] = false;
    // else
    //   Through_Stations[i][NUM_CUSTOMERS] = true;
  }

  // for (i = 0; i < 1000; i++)
  // {
  //   printf("\n");
  //   for (j = 1; j < NUM_CUSTOMERS + NUM_VEHICLES + 1; j++)
  //   {
  //     if (Routes[i][j] < NUM_CUSTOMERS)
  //     {
  //       printf("%d ", Routes[i][j]);
  //     }
  //     else
  //     {
  //       printf(" || ");
  //     }
  //   }
  //   printf("\n");
  //   for (j = 1; j < NUM_CUSTOMERS; j++)
  //   {
  //     printf("%d ", Through_Stations[i][j]);
  //   }
  // }
}

// ======= ALGORITHM ========

double compute_over_capacity(int index)
{
  int i;
  int begin_route_num = NUM_CUSTOMERS;
  double sum_capacity_over = 0.0;
  double sum_capacity_route = 0.0;
  for (i = 1; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    int node = Routes[index][i];
    if (node < begin_route_num)
    {
      sum_capacity_route += Demands[node];
    }
    else
    {
      int temp_over = MAX_CAPACITY_VH - sum_capacity_route;
      sum_capacity_route = 0.0;
      if (temp_over < 0)
        sum_capacity_over += temp_over;
    }
  }

  if (sum_capacity_over < 0.0)
  {
    Is_Feasible[index] = false;
    return -sum_capacity_over;
  }
  else
  {
    Is_Feasible[index] = true;
    return 0.0;
  }
}

double compute_cost_route(int index)
{
  int i;
  int begin_route_num = NUM_CUSTOMERS;
  double route_cost = 0.0;
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
  {
    // current node
    int node = Routes[index][i];
    if (node >= begin_route_num)
      node = 0;
    // next node
    int next_node = Routes[index][i + 1];
    if (next_node >= begin_route_num)
      next_node = 0;
    // BEGIN
    if (node != next_node)
    {
      if (Through_Stations[index][i])
      {
        double best_stat_distance = Best_Station_Distances[node][next_node];
        route_cost += best_stat_distance;
      }
      else
        route_cost += Distances[node][next_node];
    }
  }

  return route_cost;
}

double compute_over_energy(int index)
{
  int i;
  int begin_route_num = NUM_CUSTOMERS;
  bool is_feasible = true;

  double remain_eng = MAX_ENERGY_VH;
  double sum_distance = 0.0;

  double over_eng = 0.0;

  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
  {
    // current node
    int node = Routes[index][i];
    if (node >= begin_route_num)
      node = 0;

    // next node
    int next_node = Routes[index][i + 1];
    if (next_node >= begin_route_num)
      next_node = 0;

    // BEGIN
    if (Through_Stations[index][i])
    {
      double temp_over_eng = remain_eng - dist_consum(sum_distance + Distances[Best_Stations[node][next_node]][node]);
      if (temp_over_eng < 0)
      {
        is_feasible = false;
        over_eng -= abs(temp_over_eng);
      }
      else
      {
        if (!is_feasible)
        {
          over_eng -= abs(temp_over_eng);
        }
      }

      remain_eng = MAX_ENERGY_VH - dist_consum(Distances[Best_Stations[node][next_node]][next_node]);
      sum_distance = 0.0;
    }
    else
    {
      sum_distance += Distances[node][next_node];
    }

    if (next_node == 0)
    {
      if (remain_eng < 0)
        remain_eng = MAX_ENERGY_VH + remain_eng;
      else
        remain_eng = MAX_ENERGY_VH;
    }
  }

  if (!is_feasible)
    Is_Feasible[index] = false;

  return abs(over_eng);
}

/* ==== compute_H_value =====  */
double compute_H_value()
{
  int i;
  bool check = true;
  double h_s_worst = -100000.0;
  double h_s_best = 100000.0;
  for (i = 0; i < 1000; i++)
  {
    if (Is_Feasible[i])
    {
      check = false;
      if (h_s_best > Costs[i])
        h_s_best = Costs[i];
    }
    else
    {
      if (h_s_worst < Costs[i])
        h_s_worst = Costs[i];
    }
  }

  if (check)
  {
    return h_s_worst;
  }
  else
  {
    return h_s_best;
  }
}

/*==== ================ =====*/

double compute_AVG_q()
{
  int i;
  double sum_q = 0.0;
  for (i = 0; i < 1000; i++)
  {
    sum_q += Over_Capacities[i];
  }

  return sum_q / 1000;
}

double compute_AVG_e()
{
  int i;
  double sum_e = 0.0;
  for (i = 0; i < 1000; i++)
  {
    sum_e += Over_Energies[i];
  }

  return sum_e / 1000;
}

void compute_meta_data()
{
  int i;
  for (i = 0; i < 1000; i++)
  {
    double cost = compute_cost_route(i);
    double over_capacity = compute_over_capacity(i);
    double over_energy = compute_over_energy(i);

    Costs[i] = cost;
    Over_Capacities[i] = over_capacity;
    Over_Energies[i] = over_energy;
  }

  double H = compute_H_value();
  double AVG_q = compute_AVG_q();
  double AVG_e = compute_AVG_e();
  double alpha = H * (AVG_q / (AVG_q * AVG_q + AVG_e * AVG_e));
  double beta = H * (AVG_e / (AVG_q * AVG_q + AVG_e * AVG_e));

  //double fitness_val = cost + alpha * over_capacity + beta * over_energy;
  for (i = 0; i < 1000; i++)
  {
    double fitness_val = Costs[i] + alpha * Over_Capacities[i] + beta * Over_Energies[i];
    Fitness[i] = fitness_val;
  }
}

// call after Fitness compute
double virtual_fitness[1000];
int vitual_index[1000];
void bubble_sort()
{
  int i, j;
  for (i = 0; i < 1000; i++)
  {
    virtual_fitness[i] = Fitness[i];
    vitual_index[i] = i;
  }

  for (i = 0; i < 1000; i++)
  {
    for (j = i + 1; j < 1000; j++)
    {
      if (virtual_fitness[j] < virtual_fitness[i])
      {
        double temp = virtual_fitness[i];
        virtual_fitness[i] = virtual_fitness[j];
        virtual_fitness[j] = temp;
        int index_i = vitual_index[i];
        vitual_index[i] = vitual_index[j];
        vitual_index[j] = index_i;
      }
    }
  }

  // revert virtual_fitness
  for (i = 0; i < 1000 / 2; i++)
  {
    double temp = virtual_fitness[i];
    virtual_fitness[i] = virtual_fitness[999 - i];
    virtual_fitness[999 - i] = temp;
  }
}

void build_Roulette_wheel_arr()
{
  int i;
  bubble_sort();
  Roulette_Wheel_Arr[0] = virtual_fitness[0];
  for (i = 1; i < 1000; i++)
  {
    Roulette_Wheel_Arr[i] = Roulette_Wheel_Arr[i - 1] + virtual_fitness[i];
  }
}

// call after Roulette_Wheel_Arr builded
void reset_in_pool()
{
  int i;
  for (i = 0; i < 1000; i++)
  {
    Is_In_Pool[i] = false;
  }
}

void select_parent_to_pool_distinct()
{
  reset_in_pool();
  int i, j, m;
  int total = (int)Roulette_Wheel_Arr[999];
  srand((unsigned)time(NULL));
  // select 700 parent into pool
  for (i = 0; i < 700; i++)
  {
    int rand_wheel = (int)(rand() % total);
    for (j = 0; j < 1000; j++)
    {
      if (Roulette_Wheel_Arr[j] > rand_wheel)
      {
        int real_index = vitual_index[j];
        if (Is_In_Pool[real_index])
        {
          for (m = 0; m < 1000; m++)
          {
            if (!Is_In_Pool[m])
            {
              Parent_Pool[i] = m;
              Is_In_Pool[m] = true;
              break;
            }
          }
        }
        else
        {
          Parent_Pool[i] = real_index;
          Is_In_Pool[real_index] = true;
        }
        break;
      }
    }
  }
}

void select_parent_to_pool_no_distinct()
{
  int i, j, k = 0, m;
  int total = (int)Roulette_Wheel_Arr[999];
  srand((unsigned)time(NULL));
  for (i = 0; i < 700; i++)
  {
    int rand_wheel = (int)(rand() % total);
    for (j = 0; j < 1000; j++)
    {
      if (Roulette_Wheel_Arr[j] > rand_wheel)
      {
        Parent_Pool[k] = j;
        Is_In_Pool[j] = true;
        k++;
        break;
      }
    }
  }
}

// run after created pool
void reset_deprecate_data()
{
  int i;
  for (i = 0; i < 1000; i++)
  {
    Deprecate_In_Next_Gen[i] = false;
  }
}

bool check_selected_customer(int route_index, int begin, int end, int value)
{
  int i;
  for (i = begin; i <= end; i++)
  {
    if (Routes[route_index][i] == value)
      return true;
  }

  return false;
}

void compute_new_meta_data()
{
  int i;
  double H = compute_H_value();
  double AVG_q = compute_AVG_q();
  double AVG_e = compute_AVG_e();
  double alpha = H * (AVG_q / (AVG_q * AVG_q + AVG_e * AVG_e));
  double beta = H * (AVG_e / (AVG_q * AVG_q + AVG_e * AVG_e));

  for (i = 0; i < 1000; i++)
  {
    double fitness_val = Costs[i] + alpha * Over_Capacities[i] + beta * Over_Energies[i];
    Fitness[i] = fitness_val;
  }
}

void compute_is_throught_station()
{
}

double compute_cost_route_temp(int *route)
{
  int i;
  double route_cost = 0.0;
  int begin_route_num = NUM_CUSTOMERS;
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
  {
    int node = route[i];
    if (node >= begin_route_num)
      node = 0;
    int next_node = route[i + 1];
    if (next_node >= begin_route_num)
      next_node = 0;

    if (node != next_node)
    {
      route_cost += Distances[node][next_node];
    }
  }

  return route_cost;
}

int Best_ExChange[200];
void Two_Exchange_CrossOver(int index)
{
  int begin_route_num = NUM_CUSTOMERS;
  //int Best_ExChange[NUM_CUSTOMERS + NUM_VEHICLES + 1];
  // reset best_exchange
  int s, begin = 0, end = 0, i, j;
  int parent[NUM_CUSTOMERS + NUM_VEHICLES + 1];
  // capy data of parent
  for (s = 0; s < NUM_CUSTOMERS + NUM_VEHICLES + 1; s++)
  {
    parent[s] = Routes[index][s];
    Best_ExChange[s] = Routes[index][s];
  }
  double best_cost = compute_cost_route_temp(parent);
  while (true && parent[begin] < (NUM_CUSTOMERS + NUM_VEHICLES))
  {
    // find end route
    begin = end + 1;
    // find begin
    while (parent[begin] >= begin_route_num)
    {
      begin++;
    }
    end = begin + 1;
    while (parent[end] < NUM_CUSTOMERS)
    {
      end++;
    }

    if ((parent[begin] > NUM_CUSTOMERS + NUM_VEHICLES) || (parent[end] > NUM_CUSTOMERS + NUM_VEHICLES))
      break;
    // BEGIN
    for (i = begin; i < end - 2; i++)
    {
      for (j = i + 2; j < end - 1; j++)
      {
        int i_1 = i + 1;
        int j_1 = j + 1;
        int temp = parent[i_1];
        parent[i_1] = parent[j];
        parent[j] = temp;
        double route_cost = compute_cost_route_temp(parent);
        //printf("route_cost__(%0.3f, %0.3f)", best_cost, route_cost);
        if (route_cost < best_cost)
        {
          //printf("\nnew_best_route__ %0.3lf\n", route_cost);
          best_cost = route_cost;
          // copy to best_route
          for (int t = 0; t < NUM_CUSTOMERS + NUM_VEHICLES + 1; t++)
          {
            Best_ExChange[t] = parent[t];
          }
        }
        // put data back
        temp = parent[i_1];
        parent[i_1] = parent[j];
        parent[j] = temp;
      }
    }

    //END
    // end 2_exchange
    if (end == NUM_CUSTOMERS + NUM_VEHICLES)
      break;
  }
}

void Or_Exchange()
{
}

int Best_Route_Education[200];
bool Best_Route_Through_Edu[200];
void set_init_best(int *route)
{
  int i;
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    Best_Route_Education[i] = route[i];
    Best_Route_Through_Edu[i] = false;
  }
}

void exchange_education(int *route)
{
  int begin_route_num = NUM_CUSTOMERS;
  int s, begin = 0, end = 0, i, j;
  int parent[NUM_CUSTOMERS + NUM_VEHICLES + 1];

  for (s = 0; s < NUM_CUSTOMERS + NUM_VEHICLES + 1; s++)
  {
    parent[s] = route[s];
  }
  double best_cost = compute_cost_route_temp(parent);
  while (true && parent[begin] < (NUM_CUSTOMERS + NUM_VEHICLES))
  {
    begin = end + 1;
    while (parent[begin] >= begin_route_num)
    {
      begin++;
    }
    end = begin + 1;
    while (parent[end] < NUM_CUSTOMERS)
    {
      end++;
    }

    if ((parent[begin] > NUM_CUSTOMERS + NUM_VEHICLES) || (parent[end] > NUM_CUSTOMERS + NUM_VEHICLES))
      break;

    for (i = begin; i < end - 2; i++)
    {
      for (j = i + 2; j < end - 1; j++)
      {
        int i_1 = i + 1;
        int j_1 = j + 1;
        int temp = parent[i_1];
        parent[i_1] = parent[j];
        parent[j] = temp;
        double route_cost = compute_cost_route_temp(parent);

        if (route_cost < best_cost)
        {
          //printf("\nnew_best_route__ %0.3lf\n", route_cost);
          best_cost = route_cost;
          for (int t = 0; t < NUM_CUSTOMERS + NUM_VEHICLES + 1; t++)
          {
            Best_Route_Education[t] = parent[t];
          }
        }
        temp = parent[i_1];
        parent[i_1] = parent[j];
        parent[j] = temp;
      }
    }

    if (end == NUM_CUSTOMERS + NUM_VEHICLES)
      break;
  }
}

void find_through_station()
{
  int i, j;
  int first_route_num = NUM_CUSTOMERS;
  double available_eng = MAX_ENERGY_VH;
  bool is_feasible = true;
  for (j = 0; j < NUM_CUSTOMERS + NUM_VEHICLES; j++)
  {
    int node = Best_Route_Education[j];
    if (node >= first_route_num)
      node = 0;
    int next_node = Best_Route_Education[j + 1];
    if (next_node >= first_route_num)
      next_node = 0;
    if (node == 0 || node >= first_route_num)
    {
      available_eng = MAX_ENERGY_VH;
    }

    if (node != next_node)
    {
      if (is_feasible)
      {
        if (available_eng > dist_consum(Distances[node][next_node]))
        {
          available_eng -= dist_consum(Distances[node][next_node]);
          Best_Route_Through_Edu[j] = false;
          if (available_eng < 0)
            is_feasible = false;
        }
        else
        {
          int best_stat = Best_Stations[node][next_node];
          if (available_eng > dist_consum(Distances[best_stat][node]) && MAX_ENERGY_VH > dist_consum(Distances[best_stat][next_node]))
          {
            available_eng = MAX_ENERGY_VH - dist_consum(Distances[best_stat][next_node]);
            if (available_eng < 0)
              is_feasible = false;
            Best_Route_Through_Edu[j] = true;
          }
        }
      }
      else
      {
        Best_Route_Through_Edu[j] = false;
      }
    }
  }

  // printf("\nThrought_Stat: ");
  // for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
  // {
  //   printf("%d ", Best_Route_Through_Edu[i]);
  // }
}

void Education(int *route)
{
  set_init_best(route);
  exchange_education(route);
  find_through_station();
}

bool is_process[300];
void Permutation_order_1()
{
  // create child 1 and 2 space
  int i, begin_rand, end_rand;
  int num_1 = NUM_CUSTOMERS + NUM_VEHICLES + 1;
  int num = NUM_CUSTOMERS + NUM_VEHICLES;
  int child_1[num_1];
  int child_2[num_1];
  child_1[num] = 0;
  child_2[num] = 0;
  child_1[0] = 0;
  child_2[0] = 0;

  int seq_choosen[num];
  for (i = 0; i < num; i++)
    seq_choosen[i] = i;

  srand((unsigned)time(NULL));
  int count_pool = -1;
  //Through_Stations_In_Pool
  for (int s = 0; s < 700 / 2; s++)
  {
    // select parent
    int parent_1 = (int)(rand() % (700 - s * 2));
    int temp = Parent_Pool[parent_1];
    Parent_Pool[parent_1] = Parent_Pool[699 - s * 2];
    Parent_Pool[699 - s * 2] = temp;
    parent_1 = Parent_Pool[699 - s * 2];

    int parent_2 = (int)(rand() % (700 - s * 2 - 1));
    temp = Parent_Pool[parent_2];
    Parent_Pool[parent_2] = Parent_Pool[699 - s * 2 - 1];
    Parent_Pool[699 - s * 2 - 1] = temp;
    parent_2 = Parent_Pool[699 - s * 2 - 1];

    // in permutation
    // CHILD 1
    begin_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 1)) + 1;
    temp = seq_choosen[begin_rand];
    seq_choosen[begin_rand] = seq_choosen[NUM_CUSTOMERS + NUM_VEHICLES - 1];
    seq_choosen[NUM_CUSTOMERS + NUM_VEHICLES - 1] = temp;
    begin_rand = seq_choosen[NUM_CUSTOMERS + NUM_VEHICLES - 1];

    end_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 2)) + 1;
    temp = seq_choosen[end_rand];
    seq_choosen[end_rand] = seq_choosen[NUM_CUSTOMERS + NUM_VEHICLES - 2];
    seq_choosen[NUM_CUSTOMERS + NUM_VEHICLES - 2] = temp;
    end_rand = seq_choosen[NUM_CUSTOMERS + NUM_VEHICLES - 2];

    if (begin_rand > end_rand)
    {
      temp = begin_rand;
      begin_rand = end_rand;
      end_rand = temp;
    }

    for (i = begin_rand; i <= end_rand; i++)
    {
      child_1[i] = Routes[parent_1][i];
    }

    int j = 1;
    for (i = 1; i < begin_rand; i++)
    {
      for (; j < NUM_CUSTOMERS + NUM_VEHICLES; j++)
      {
        if (!check_selected_customer(parent_1, begin_rand, end_rand, Routes[parent_2][j]))
        {
          child_1[i] = Routes[parent_2][j];
          j++;
          break;
        }
      }
    }

    for (i = end_rand + 1; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
    {
      for (; j < NUM_CUSTOMERS + NUM_VEHICLES; j++)
      {
        if (!check_selected_customer(parent_1, begin_rand, end_rand, Routes[parent_2][j]))
        {
          child_1[i] = Routes[parent_2][j];
          j++;
          break;
        }
      }
    }

    child_1[NUM_CUSTOMERS + NUM_VEHICLES] = 0;

    // CHILD 2
    int rand_choose_parent = (int)(rand() % 2);
    int select_parent = parent_1;
    if (rand_choose_parent == 1)
      select_parent = parent_2;

    // this is child 2 in Best_ExChange
    Two_Exchange_CrossOver(select_parent);
    for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
    {
      child_2[i] = Best_ExChange[i];
    }
    // education
    Education(child_1);
    count_pool++;
    for (int k = 0; k < NUM_CUSTOMERS + NUM_VEHICLES + 1; k++)
    {
      Route_In_Pool[count_pool][k] = Best_Route_Education[k];
      if (k < NUM_CUSTOMERS + NUM_VEHICLES)
      {
        Through_Stations_In_Pool[count_pool][k] = Best_Route_Through_Edu[k];
      }
    }
    Education(child_2);
    count_pool++;
    for (int k = 0; k < NUM_CUSTOMERS + NUM_VEHICLES + 1; k++)
    {
      Route_In_Pool[count_pool][k] = Best_Route_Education[k];
      if (k < NUM_CUSTOMERS + NUM_VEHICLES)
      {
        Through_Stations_In_Pool[count_pool][k] = Best_Route_Through_Edu[k];
      }
    }
    // mutation for child 2
  }

  // for (int s = 0; s < 700; s++)
  // {
  //   printf("\nroute %d: ", s);
  //   for (int n = 0; n < NUM_CUSTOMERS + NUM_VEHICLES + 1; n++)
  //   {
  //     printf("%d ", Route_In_Pool[s][n]);
  //   }
  // }

  for (int s = 0; s < 300; s++)
  {
    is_process[s] = false;
    int index = vitual_index[s];
    if (index < 300)
      is_process[index] = true;
  }

  for (int s = 0; s < 300; s++)
  {
    int index = vitual_index[s];
    if (index > 300)
    {
      for (int k = 0; k < 300; k++)
      {
        if (!is_process[k])
        {
          for (int m = 0; m < NUM_CUSTOMERS + NUM_VEHICLES; m++)
          {
            Routes[k][m] = Routes[index][m];
            if (m < NUM_CUSTOMERS + NUM_VEHICLES)
            {
              Through_Stations[k][m] = Through_Stations[index][m];
            }
          }
          break;
        }
      }
    }
  }

  int count = -1;
  for (int s = 300; s < 1000; s++)
  {
    count++;
    for (int m = 0; m < NUM_CUSTOMERS + NUM_VEHICLES + 1; m++)
    {
      Routes[s][m] = Route_In_Pool[count][m];
      if (m < NUM_CUSTOMERS + NUM_VEHICLES)
      {
        Through_Stations[m] = Through_Stations_In_Pool[m];
      }
    }
  }
}

void tune_result()
{
  int i, j;
  int first_route_num = NUM_CUSTOMERS;
  for (i = 0; i < 1000; i++)
  {
    double available_eng = MAX_ENERGY_VH;
    for (j = 0; j < NUM_CUSTOMERS + NUM_VEHICLES; j++)
    {
      int node = Routes[i][j];
      if (node >= first_route_num)
        node = 0;
      int next_node = Routes[i][j + 1];
      if (next_node >= first_route_num)
        next_node = 0;
      if (node == 0)
      {
        available_eng = MAX_ENERGY_VH;
      }

      if (node != next_node)
      {
        if (available_eng > dist_consum(Distances[node][next_node]))
        {
          available_eng -= dist_consum(Distances[node][next_node]);
          Through_Stations[i][j] = false;
        }
        else
        {
          int best_stat = Best_Stations[node][next_node];
          if (available_eng > dist_consum(Distances[best_stat][node]) && MAX_ENERGY_VH > dist_consum(Distances[best_stat][next_node]))
          {
            available_eng = MAX_ENERGY_VH - dist_consum(Distances[best_stat][next_node]);
            Through_Stations[i][j] = true;
          }
          else
          {
            Is_Feasible[i] = false;
            break;
          }
        }
      }
    }
  }
}

int main()
{
  auto start = high_resolution_clock::now();



  FILE *fp;
  fp = fopen("./result.txt", "a");
  //fprintf(fp, "\n--------------------------------------------------------------------------------\n");
  int i;
  read_file("E-n101-k8.evrp");
  fprintf(fp, "\n============================================================================\n");
  fprintf(fp, "\n\n-->data: dimention: %d - num_customer: %d - capacity_vh: %lf - energy_vh: %lf - energy_consumtion: %lf - num_vehicles: %d\n", DIMENTION, NUM_CUSTOMERS, MAX_CAPACITY_VH, MAX_ENERGY_VH, ENG_CONSUMTION, NUM_VEHICLES);
  prepare_data();
  init_population();
  compute_meta_data();

  for (i = 0; i < 1000; i++)
  {
    build_Roulette_wheel_arr();
    select_parent_to_pool_distinct();
    Permutation_order_1();
    tune_result();
    compute_meta_data();
  }

  int best_route = 0;
  double best_cost = Costs[0];
  for (i = 0; i < 1000; i++)
  {
    if (best_cost > Costs[i])
    {
      best_cost = Costs[i];
      best_route = i;
    }
  }

  printf("\nBEST ROUTE FOUND: %d - Cost: %0.3lf - optimal: %.3lf", best_route, best_cost, OPTIMAL_VALUE);
  printf("\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    printf("%d -> ", Routes[best_route][i]);
  }
  printf("\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    printf("%d -> ", Through_Stations[best_route][i]);
  }

  fprintf(fp, "\nLần 6: BEST ROUTE FOUND: %d - Cost: %0.3lf - optimal: %.3lf", best_route, best_cost, OPTIMAL_VALUE);
  fprintf(fp, "\nRoute: ");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Routes[best_route][i]);
  }
  fprintf(fp, "\nThrought_station: ");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Through_Stations[best_route][i]);
  }
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  cout << "\nTime taken by function: "
         << duration.count()/1000 << " milliseconds" << endl;
  fprintf(fp,"\ntime_run: %llu milliseconds", duration.count()/1000);
  fclose(fp);
}