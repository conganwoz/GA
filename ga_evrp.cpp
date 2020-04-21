#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <time.h>

using namespace std;

// -- data problem --
int DIMENTION = 0;
int NUM_STATIONS = 0;
int NUM_CUSTOMERS = 0;
// int NUM_SAVING_DISTANCES = 0;
int NUM_VEHICLES = 0;

double MAX_CAPACITY_VH = 0.0;
double MAX_ENERGY_VH = 0.0;
double ENG_CONSUMTION = 0.0;
double OPTIMAL_VALUE = 0.0;

double **Distances;
// double Saving_Distances[1000][3];
double Demands[2000];
double **Coords;

int **Best_Stations;
double **Best_Station_Distances;

double **F_CAP;
double **B_CAP;
double **F_ENERGY;
double **B_ENERGY;

int **Routes;
bool **Through_Stations;
double *Fitnesses;
double Alpha, Beta, H;
double Roulette_Wheel_Arr[1000];
int Parent_Pool[500];
bool Is_In_Pool[1000];
bool Deprecate_In_Next_Gen[1000];

int *Temp_Sequence_Customers;

// Meta data
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
  Through_Stations = (bool **)malloc(1000 * sizeof(bool *));

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

  printf("data: dimention: %d - num_customer: %d - capacity_vh: %lf - energy_vh: %lf - energy_consumtion: %lf - num_vehicles: %d\n", DIMENTION, NUM_CUSTOMERS, MAX_CAPACITY_VH, MAX_ENERGY_VH, ENG_CONSUMTION, NUM_VEHICLES);
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

  // for (int i = 0; i < DIMENTION; i++)
  // {
  //   for (int j = i + 1; j < DIMENTION; j++)
  //   {
  //     printf("D(%d, %d) = %lf\n", i, j, Distances[i][j]);
  //   }
  // }
}

double dist_consum(double distance)
{
  return distance * ENG_CONSUMTION;
}

void init_population()
{
  int i, j, k, rand_through;
  srand((unsigned)time(NULL));
  int num_customer_each_route = (int)((NUM_CUSTOMERS - 1) / NUM_VEHICLES) + 1;
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

    rand_through = (int)(rand() % 2);
    if (rand_through == 0)
      Through_Stations[i][NUM_CUSTOMERS] = false;
    else
      Through_Stations[i][NUM_CUSTOMERS] = true;
  }

  // for (i = 0; i < 1000; i++)
  // {
  //   printf("\n");
  //   for (j = 1; j < NUM_CUSTOMERS + NUM_VEHICLES + 1; j++)
  //   {
  //     printf("%d ", Routes[i][j]);
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
  int begin_route_num = NUM_CUSTOMERS - 1;
  double sum_capacity = 0.0;
  for (i = 1; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
  {
    int node = Routes[index][i];
    if (node < begin_route_num)
    {
      sum_capacity += Demands[node];
    }
  }

  double over_capacity = MAX_CAPACITY_VH - sum_capacity;
  if (over_capacity < 0.0)
  {
    Is_Feasible[index] = false;
    return -over_capacity;
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
  // // begin
  // int begin_node = Routes[index][1];
  // //
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

double compute_cost_route_debug(int index)
{
  int i;
  int begin_route_num = NUM_CUSTOMERS;
  double route_cost = 0.0;
  // // begin
  // int begin_node = Routes[index][1];
  // //
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

    if (node != next_node)
    {
      if (Through_Stations[index][i])
      {
        double best_stat_distance = Best_Station_Distances[node][next_node];
        printf("\ncost(%d, %d): %0.3lf - %0.3lf", node, next_node, route_cost, best_stat_distance);
        route_cost += best_stat_distance;
      }
      else
      {
        printf("\ncost(%d, %d): %0.3lf - %0.3lf", node, next_node, route_cost, Distances[node][next_node]);
        route_cost += Distances[node][next_node];
      }
    }
  }

  printf("\nfinal: %0.3lf", route_cost);

  return route_cost;
}

double compute_over_energy(int index)
{
  int i;
  bool is_feasible = true;
  int begin_route_num = NUM_CUSTOMERS - 1;
  double remain_energy = MAX_ENERGY_VH;
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

    if (is_feasible)
    {
      if (node != next_node)
      {
        if (Through_Stations[index][i])
        {
          int best_stat = Best_Stations[node][next_node];
          double eng_stat = remain_energy - dist_consum(Distances[node][best_stat]);
          if (eng_stat < 0)
          {
            is_feasible = false;
            remain_energy = MAX_ENERGY_VH + eng_stat - dist_consum(Distances[best_stat][next_node]);
            if(remain_energy < 0) is_feasible = false;
          }
          else
          {
            remain_energy = MAX_ENERGY_VH - dist_consum(Distances[best_stat][next_node]);
            if(remain_energy < 0) is_feasible = false;
          }
        }
        else
        {
          remain_energy -= dist_consum(Distances[node][next_node]);
          if(remain_energy < 0) is_feasible = false;
        }
      }
    } else {
      if (node != next_node)
      {
        if (Through_Stations[index][i])
        {
          int best_stat = Best_Stations[node][next_node];
          remain_energy -= dist_consum(Best_Station_Distances[node][next_node]);
        } else
        {
          remain_energy -= dist_consum(Distances[node][next_node]);
        }
      }
    }
  }

  if (is_feasible)
  {
    Is_Feasible[index] = true;
    return 0.0;
  }
  else
  {
    Is_Feasible[index] = false;
    if (abs(remain_energy) < 10.0)
      return MAX_ENERGY_VH;
    else
      return abs(remain_energy);
  }
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

  // for (i = 0; i < 1000; i++)
  // {
  //   printf(" %0.3lf - cost: %lf - is_feasible: %d\n", Fitness[i], Costs[i], Is_Feasible[i]);
  // }
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
  // int i;
  // Roulette_Wheel_Arr[0] = Fitness[0];
  // for (i = 1; i < 1000; i++)
  // {
  //   Roulette_Wheel_Arr[i] = Roulette_Wheel_Arr[i - 1] + Fitness[i];
  // }
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
  for (i = 0; i < 500; i++)
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

  // for (i = 0; i < 500; i++)
  // {
  //   printf("pool: %d\n", Parent_Pool[i]);
  // }
}

void select_parent_to_pool_no_distinct()
{
  int i, j, k = 0, m;
  int total = (int)Roulette_Wheel_Arr[999];
  srand((unsigned)time(NULL));
  for (i = 0; i < 500; i++)
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

  // for (i = 0; i < 500; i++)
  // {
  //   printf("pool: %d\n", Parent_Pool[i]);
  // }
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

void Permutation_order_1()
{
  int i, begin_rand, end_rand;
  int num_1 = NUM_CUSTOMERS + NUM_VEHICLES + 1;
  int num = NUM_CUSTOMERS + NUM_VEHICLES;
  // int child_1[NUM_CUSTOMERS + NUM_VEHICLES + 1];
  // int child_2[NUM_CUSTOMERS + NUM_VEHICLES + 1];
  int child_1[num_1];
  int child_2[num_1];
  child_1[num] = 0;
  child_2[num] = 0;
  child_1[0] = 0;
  child_2[0] = 0;
  int seq_choosen[num];
  for (i = 0; i < num; i++)
    seq_choosen[i] = i;

  //reset_deprecate_data();
  srand((unsigned)time(NULL));
  for (int s = 0; s < 498 / 2; s++)
  {
    // select parent
    int parent_1 = (int)(rand() % (500 - s * 2));
    int temp = Parent_Pool[parent_1];
    Parent_Pool[parent_1] = Parent_Pool[499 - s * 2];
    Parent_Pool[499 - s * 2] = temp;
    parent_1 = Parent_Pool[499 - s * 2];

    int parent_2 = (int)(rand() % (500 - s * 2 - 1));
    temp = Parent_Pool[parent_2];
    Parent_Pool[parent_2] = Parent_Pool[499 - s * 2 - 1];
    Parent_Pool[499 - s * 2 - 1] = temp;
    parent_2 = Parent_Pool[499 - s * 2 - 1];

    // in permutation
    // child 1
    begin_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 1)) + 1;
    temp = seq_choosen[begin_rand];
    seq_choosen[begin_rand] = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 1];
    seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 1] = temp;
    begin_rand = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 1];

    end_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 2)) + 1;
    temp = seq_choosen[end_rand];
    seq_choosen[end_rand] = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 2];
    seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 2] = temp;
    end_rand = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 2];

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

    // child 2
    for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
      seq_choosen[i] = i;
    begin_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 1)) + 1;
    temp = seq_choosen[begin_rand];
    seq_choosen[begin_rand] = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 1];
    seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 1] = temp;
    begin_rand = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 1];

    end_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 2)) + 1;
    temp = seq_choosen[end_rand];
    seq_choosen[end_rand] = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 2];
    seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 2] = temp;
    end_rand = seq_choosen[NUM_CUSTOMERS + NUM_CUSTOMERS - 2];

    if (begin_rand > end_rand)
    {
      temp = begin_rand;
      begin_rand = end_rand;
      end_rand = temp;
    }

    for (i = begin_rand; i <= end_rand; i++)
    {
      child_2[i] = Routes[parent_2][i];
    }

    j = 1;
    for (i = 1; i < begin_rand; i++)
    {
      for (; j < NUM_CUSTOMERS + NUM_VEHICLES; j++)
      {
        if (!check_selected_customer(parent_2, begin_rand, end_rand, Routes[parent_1][j]))
        {
          child_2[i] = Routes[parent_1][j];
          j++;
          break;
        }
      }
    }

    for (i = end_rand + 1; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
    {
      for (; j < NUM_CUSTOMERS + NUM_VEHICLES; j++)
      {
        if (!check_selected_customer(parent_2, begin_rand, end_rand, Routes[parent_1][j]))
        {
          child_2[i] = Routes[parent_1][j];
          j++;
          break;
        }
      }
    }
    child_2[NUM_CUSTOMERS + NUM_VEHICLES] = 0;

    // mutation child 2

    begin_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 1)) + 1;
    end_rand = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 2)) + 1;
    temp = child_2[begin_rand];
    child_2[begin_rand] = child_2[end_rand];
    child_2[end_rand] = temp;

    // // produce new population
    //Routes[parent_1] = child_1;
    for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
    {
      Routes[parent_1][i] = child_1[i];
    }
    // permutation for Thought Station child 1
    int point = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 1)) + 1;
    for (i = point; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
    {
      Through_Stations[parent_1][i] = Through_Stations[parent_2][i];
    }
    double cost = compute_cost_route(parent_1);
    double over_capacity = compute_over_capacity(parent_1);
    double over_energy = compute_over_energy(parent_1);

    Costs[parent_1] = cost;
    Over_Capacities[parent_1] = over_capacity;
    Over_Energies[parent_1] = over_energy;

    //Routes[parent_2] = child_2;
    for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES; i++)
    {
      Routes[parent_2][i] = child_2[i];
    }
    // permutation for Thought Station child 2
    point = (int)(rand() % (NUM_CUSTOMERS + NUM_VEHICLES - 1)) + 1;
    for (i = 0; i < point; i++)
    {
      Through_Stations[parent_2][i] = Through_Stations[parent_1][i];
    }
    cost = compute_cost_route(parent_2);
    over_capacity = compute_over_capacity(parent_2);
    over_energy = compute_over_energy(parent_2);

    Costs[parent_2] = cost;
    Over_Capacities[parent_2] = over_capacity;
    Over_Energies[parent_2] = over_energy;

    //compute new meta data
    compute_new_meta_data();
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
  FILE *fp;
  fp = fopen("./result.txt", "a");
  fprintf(fp, "\n============================================================================\n");
  int i;
  read_file("E-n30-k3.evrp");
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
  }
  //tune_result();

  int best_route = 0;
  double best_cost = Costs[0];
  for (i = 0; i < 1000; i++)
  {
    if (best_cost > Costs[i])
    {
      best_cost = Costs[i];
      best_route = i;
    }
    // printf(" %0.3lf - cost: %lf - is_feasible: %d\n", Fitness[i], Costs[i], Is_Feasible[i]);
  }

  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Routes[best_route][i]);
  }
  printf("\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Through_Stations[best_route][i]);
  }

  fprintf(fp, "\nBEST ROUTE FOUND: %d - Cost: %0.3lf - optimal: %.3lf", best_route, best_cost, OPTIMAL_VALUE);
  //compute_cost_route_debug(best_route);

  //////////////////////////////////////////////////////////////////////////////
  read_file("E-n23-k3.evrp");
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
  }
  //tune_result();

  best_route = 0;
  best_cost = Costs[0];
  for (i = 0; i < 1000; i++)
  {
    if (best_cost > Costs[i])
    {
      best_cost = Costs[i];
      best_route = i;
    }
    // printf(" %0.3lf - cost: %lf - is_feasible: %d\n", Fitness[i], Costs[i], Is_Feasible[i]);
  }

  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Routes[best_route][i]);
  }
  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Through_Stations[best_route][i]);
  }

  fprintf(fp, "\nBEST ROUTE FOUND: %d - Cost: %0.3lf - optimal: %0.3lf", best_route, best_cost, OPTIMAL_VALUE);

  ////////////////////////////////////////////////////////////////////////////////
  read_file("E-n76-k7.evrp");
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
  }
  //tune_result();

  best_route = 0;
  best_cost = Costs[0];
  for (i = 0; i < 1000; i++)
  {
    if (best_cost > Costs[i])
    {
      best_cost = Costs[i];
      best_route = i;
    }
    // printf(" %0.3lf - cost: %lf - is_feasible: %d\n", Fitness[i], Costs[i], Is_Feasible[i]);
  }

  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Routes[best_route][i]);
  }
  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Through_Stations[best_route][i]);
  }

  fprintf(fp, "\nBEST ROUTE FOUND: %d - Cost: %0.3lf - optimal: %0.3lf", best_route, best_cost, OPTIMAL_VALUE);

  ////////////////////////////////////////////////////////////////////////////////

  read_file("E-n22-k4.evrp");
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
  }

  best_route = 0;
  best_cost = Costs[0];
  for (i = 0; i < 1000; i++)
  {
    if (best_cost > Costs[i])
    {
      best_cost = Costs[i];
      best_route = i;
    }
    // printf(" %0.3lf - cost: %lf - is_feasible: %d\n", Fitness[i], Costs[i], Is_Feasible[i]);
  }

  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Routes[best_route][i]);
  }
  fprintf(fp, "\n");
  for (i = 0; i < NUM_CUSTOMERS + NUM_VEHICLES + 1; i++)
  {
    fprintf(fp, "%d -> ", Through_Stations[best_route][i]);
  }

  fprintf(fp, "\nBEST ROUTE FOUND: %d - Cost: %0.3lf - optimal: %0.3lf", best_route, best_cost, OPTIMAL_VALUE);

  fclose(fp);

  return -1;
}
