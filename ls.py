import sys, time, pickle, threading
from socket import *
import heapq

#print("File one __name__ is set to: {}" .format(__name__))


def costchange(tempgraph, interval):
    time.sleep(1.2 * int(interval))
    other_port[local_port] += 1
    templsa = lsa
    templsa['graph'] = tempgraph
    for port in graph.keys():
        data = pickle.dumps(lsa)
        skt.sendto(data, ('localhost', port))
        other_port[local_port] = lsa['num']
        print('[{}] LSA of Node {} with sequence number {} sent to Node {}'.format(time.time(), local_port, lsa['num'],
                                                                                   port))



def dijkstra(lst, local_port):
    q = []
    weight = {}
    p = {}
    heapq.heappush(q, (0, local_port))
    weight[local_port] = 0
    for i in lst.keys():
        if i != local_port:
            weight[i] = float('inf')
            p[i] = None
    #print(weight)
    while q:
        u = heapq.heappop(q)[1]
        for v in lst[u].keys():
            #print(u, v)
            if v not in weight:
                weight[v] = float('inf')
            if weight[v] > weight[u] + lst[u][v]:
                weight[v] = weight[u] + lst[u][v]
                p[v] = u
                heapq.heappush(q, (weight[v], v))
    return weight, p


def listen(lst, local_port, other_port):
    while True:
        msg, addr = skt.recvfrom(2048)
        temp = pickle.loads(msg)
        sender = temp['port']
        new = temp['graph']
        num = int(temp['num'])
        print('[{}] LSA of Node {} with sequence number {} recieved from Node {}'.format(time.time(), local_port, num,
                                                                                   addr[1]))
        if sender not in other_port or other_port[sender] < num:
            other_port[sender] = num
            for port in lst[local_port]:
                skt.sendto(msg, ('localhost', port))
                print('[{}] LSA of Node {} with sequence number {} sent to Node {}'.format(time.time(), addr[1], num, port))
        else:
            print('[{}] DUPLICATE LSA packet Received, AND DROPPED'.format(time.time()))
            print('- LSA of node {}'.format(sender))
            print('- Recieved from {}'.format(addr[1]))

        if sender not in lst or lst[sender] != new:
            lst[sender] = new
            for node in lst[sender]:
                #print(lst, node)
                if node not in lst:
                    lst[node] = {}
                lst[node][sender] = lst[sender][node]
            break
    return lst, sender, other_port

def send(lsa, interval, lastNode):
    #n = 1
    while True:
        lsa['num'] += 1
        for port in graph.keys():
            data = pickle.dumps(lsa)
            skt.sendto(data, ('localhost', port))
            other_port[local_port] = lsa['num']
            print('[{}] LSA of Node {} with sequence number {} sent to Node {}'.format(time.time(), local_port, lsa['num'], port))
            time.sleep(int(interval))
        #n+=1



if __name__ == "__main__":
    #print(sys.argv)
    try:
        interval = sys.argv[2]
        local_port = int(sys.argv[3])
        other_port = {}
        if local_port <= 1024:
            print('Invalid local-port number')
            exit()
        graph = {}
        lastNode = False
        # graph[local_port] = 0
        i = 4
        while i < len(sys.argv):
            if sys.argv[i] == 'last':
                lastNode = True
                i += 1
                cost = 0
                if i < len(sys.argv):
                    cost = int(sys.argv[i])
                break
            if int(sys.argv[i]) <= 1024:
                print('Invalid local-port number' + sys.argv[i])
                exit()
            graph[int(sys.argv[i])] = int(sys.argv[i + 1])
            i += 2
        graph[local_port] = 0
        # print(graph)
        pacnum = 0
        lsa = {}
        lsa['port'] = local_port
        lsa['graph'] = graph
        lsa['num'] = int(str(local_port) + str(pacnum))
        pacnum += 1

        lst = {}
        lst[local_port] = graph
        for port in graph:
            if port != local_port:
                lst[port] = {port: 0, local_port: lst[local_port][port]}
        # print(lst)
        print('[{}] Node {} Network topology'.format(time.time(), local_port))
        for i in sorted(lst[local_port].keys()):
            print('- {} from Node {} to Node {}'.format(lst[local_port][i], local_port, i))

        networkTable, p = dijkstra(lst, local_port)
        print('[{}] Node {} Routing Table'.format(time.time(), local_port))
        for i in sorted(networkTable.keys()):
            print('- {} Node {} '.format(networkTable[i], i))
        # print(networkTable,1)

        skt = socket(AF_INET, SOCK_DGRAM)
        skt.bind(("", local_port))

        if lastNode:
            for port in graph.keys():
                if port != local_port:
                    data = pickle.dumps(lsa)
                    skt.sendto(data, ('localhost', port))
                    other_port[local_port] = int(lsa['num'])
                    print('[{}] LSA of Node {} with sequence number {} sent to Node {}'.format(time.time(), local_port,
                                                                                               lsa['num'], port))

        lst, newport, other_port = listen(lst, local_port, other_port)
        #print(lst)
        networkTable, p = dijkstra(lst, local_port)
        print('[{}] DUPLICATE LSA packet Received, AND DROPPED'.format(time.time()))
        print(networkTable)

        th = threading.Thread(target=send, args=(lsa, interval, lastNode))
        th.start()

        if lastNode:
            maxi = 1024
            for port in graph:
                if port != local_port:
                    maxi = max(port, maxi)
            tempgraph = graph
            tempgraph[maxi] = cost

            costchange(graph, interval)

        while True:
            try:
                lst, newport, other_port = listen(lst, local_port, other_port)
                # print(lst)
                print('[{}] Node {} Network topology'.format(time.time(), local_port))
                for i in sorted(lst[local_port].keys()):
                    print('- {} from Node {} to Node {}'.format(lst[local_port][i], local_port, i))
                networkTable, p = dijkstra(lst, local_port)
                print(p)
                print('[{}] Node {} Routing Table'.format(time.time(), local_port))
                for i in sorted(networkTable.keys()):
                    if i != local_port:
                        if p[i] == local_port:
                            print('- {} -> Node {} '.format(networkTable[i], i))
                        else:
                            j = i
                            while p[j] != local_port:
                                j = p[j]
                            print('- {} -> Node {} ; Next hop -> Node {}'.format(networkTable[i], i, j))
            except KeyboardInterrupt:
                exit()
        #print(networkTable)

        #other_port[newport] += 1
    #
    #print(newlst, newport)
    except KeyboardInterrupt:
        exit()



