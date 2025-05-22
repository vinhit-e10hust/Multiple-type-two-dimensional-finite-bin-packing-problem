from ortools.linear_solver import pywraplp
import sys 
import time

def Input():
  data = {}
  f = open("0010.txt", 'r')
  [N,K] = [int(x) for x in f.readline().split()]
  item_sizes = []
  truck_sizes = []
  truck_costs = []
  
  for i in range(N):
    item_sizes.append(list(map(int,f.readline().split())))
  for i in range(K):
    wi, hi, ci = list(map(int,f.readline().split()))
    truck_sizes.append([wi , hi])
    truck_costs.append(ci) 
  
  return N,K,item_sizes,truck_sizes,truck_costs

N,K,item_sizes,truck_sizes,truck_costs = Input()



def main_solver1():
  start_time = time.time()
  solver = pywraplp.Solver.CreateSolver('SCIP')
  if not solver:
    return

  M = 1000000000
  # Tạo dict lưu trữ biến
  x = {}
  Rot = {}
  l = {}
  r = {}
  t = {}
  b = {}

  max_width = max(truck[0] for truck in truck_sizes)
  max_length = max(truck[1] for truck in truck_sizes)

  #Khai bao bien
  for i in range(N):
    for j in range(K):
      x[i,j] = solver.IntVar(0,1,f'x[{i},{j}]')

  # Mỗi vật để duy nhất trên 1 xe
  for i in range(N):
    solver.Add(solver.Sum(x[i,j] for j in range(K)) == 1)

  #Constraint
  for i in range(N):
    Rot[i] = solver.IntVar(0,1,f'Rot[{i}]')
    l[i] = solver.IntVar(0,max_width,f'l[{i}]')
    r[i] = solver.IntVar(0,max_width,f'r[{i}]')
    t[i] = solver.IntVar(0,max_length,f't[{i}]')
    b[i] = solver.IntVar(0,max_length,f'b[{i}]')

    solver.Add(r[i] == l[i] + (1 - Rot[i]) * item_sizes[i][0] + Rot[i] * item_sizes[i][1])
    solver.Add(t[i] == b[i] + (1 - Rot[i]) * item_sizes[i][1] + Rot[i] * item_sizes[i][0])

    for j in range(K):
      solver.Add(r[i] <= (1 - x[i,j])* M + truck_sizes[j][0])
      solver.Add(l[i] <= (1 - x[i,j])* M + truck_sizes[j][0])
      solver.Add(t[i] <= (1 - x[i,j])* M + truck_sizes[j][1])
      solver.Add(b[i] <= (1 - x[i,j])* M + truck_sizes[j][1])

  # Đảm bảo các vật phẩm không bị đè lên nhau
  for i in range(N):
    for j in range(i+1 ,N):
      for k in range(K):

        # Khởi tạo biến e
        e = solver.IntVar(0,1,f'e[i,j]')
        solver.Add(e <= x[i,k])
        solver.Add(e <= x[j,k])
        solver.Add(e >= x[i,k] + x[j,k] - 1)

        # Khởi tạo các điều kiện để 2 vật trong 1 xe không bị đè lên nhau
        a1 = solver.IntVar(0,1,f'a1[i,j]')
        a2 = solver.IntVar(0,1,f'a2[i,j]')
        a3 = solver.IntVar(0,1,f'a3[i,j]')
        a4 = solver.IntVar(0,1,f'a4[i,j]')

        solver.Add(r[i] <= l[j] + M * (1 - a1))  # Item i is to the left of item j
        solver.Add(r[j] <= l[i] + M * (1 - a2))  # Item j is to the left of item i
        solver.Add(t[i] <= b[j] + M * (1 - a3))  # Item i is below item j
        solver.Add(t[j] <= b[i] + M * (1 - a4))  # Item j is below item i

        # At least one non-overlapping condition must be true
        solver.Add(a1 + a2 + a3 + a4 <= e*M)
        solver.Add(a1 + a2 + a3 + a4 + (1-e)*M >= 1)

  # Tìm xe được sử dụng
  z = {}
  for j in range(K):
    z[j] = solver.IntVar(0, 1, f'z[{j}]')

    # Constraint to link the usage of truck j with the items placed in it
    q = solver.IntVar(0, N, f'q[{j}]')
    solver.Add(q == solver.Sum(x[i, j] for i in range(N)))

    solver.Add(z[j] <= q * M)
    solver.Add(z[j] * M >= q)

  # Hàm mục tiêu
  total_cost = solver.Sum(z[j] * truck_costs[j] for j in range(K))
  solver.Minimize(total_cost)
  status = solver.Solve()

  if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
    res = []
    for i in range(N):
      for j in range(K):
        if x[i, j].solution_value() == 1:
          rotation = Rot[i].solution_value()
          res.append([i+1 , j+1 , int(l[i].solution_value()) , int(b[i].solution_value()) , int(rotation)])

    for line in res:
      print(" ".join(map(str,line)))

    print(int(total_cost.solution_value()))

    end_time = time.time()
    diff = end_time - start_time
    print(f'{diff:.9f} sec')
  else:
    print('No solution.')


if __name__ == "__main__":
  main_solver1()
