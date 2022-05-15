import sys, time, pickle, threading
from socket import *
import heapq

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


def listenp(routingTable, cost):
    graph = cost
    sendingTime = 0
    mode = False
    n = 0
    num = 0
    while True:
        graph = cost
        sendingTime = 0
        mode = False
        n = 0
        num = 0
        while True:
            msg, addr = skt.recvfrom(2048)
            temp = pickle.loads(msg)
            try:
                temp[3]
                mode = True
                continue
            except:
                xx = False
            try:
                graph[addr[1]] = temp[2]
                continue
            except:
                newTable = temp[addr[1]]
            flag = 0
            #print(temp)
            # print(newTable)
            print('[{}] Message received at Node {} from Node {}'.format(time.time(), local_port, addr[1]))
            # print(pickle.loads(msg), addr)
            update = False
            # In every table: change -> only the line which represent the port changed. every other lines changed not count.
            # put new line into routing table

            # whether there are new nodes
            if mode == True:
                #print(1)
                #print(routingTable[addr[1]], newTable)
                for port in newTable:
                    if routingTable[addr[1]][port] != newTable[port]:
                        routingTable[addr[1]][port] = newTable[port]
                        routingTable[port][addr[1]] = newTable[port]
                        update = True

                if update:
                    for node in routingTable[local_port].keys():
                        if node != local_port:
                            routingTable[local_port][node][1] = float('inf')
                            for port in graph:
                                if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                                    routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                                    routingTable[local_port][node][0] = port
                                    if port != node:
                                        temp[port] = node

                    print('[{}] Node {} Routing Table'.format(time.time(), local_port))
                    for port in routingTable[local_port]:
                        if port == local_port:
                            continue
                        if port != routingTable[local_port][port][0]:
                            print(
                                '- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                                routingTable[local_port][port][0]))
                        else:
                            print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

                    for port in graph:
                        if port != local_port:
                            table = routingTable
                            if num < 1:
                                if port != routingTable[local_port][port][0]:
                                    # print(table[local_port], port, temp[addr[1]])
                                    table[local_port][port] = [port, float('inf')]
                                data = pickle.dumps(table)
                                skt.sendto(data, ('localhost', port))
                                num += 1
                            else:
                                if port != routingTable[local_port][port][0]:
                                    data = pickle.dumps(routingTable)
                                    skt.sendto(data, ('localhost', port))

            if mode == False:
                for port in newTable.keys():
                    if port not in routingTable[local_port]:
                        routingTable[port] = {}
                        for node in routingTable.keys():
                            routingTable[node][port] = [None, float('inf')]
                            routingTable[port][node] = [None, float('inf')]
                        update = True
                        # print(update)

                # routingTable[addr[1]] = newTable
                for port in newTable:
                    if routingTable[addr[1]][port] != newTable[port]:
                        routingTable[addr[1]][port] = newTable[port]
                        update = True

                # print(routingTable)

                # update
                for node in routingTable[local_port].keys():
                    if node != local_port:
                        routingTable[local_port][node][1] = float('inf')
                        for port in graph:
                            if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                                routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                                routingTable[local_port][node][0] = port
                                # print(port)
                                # update = True

                # print table
                print('[{}] Node {} Routing Table'.format(time.time(), local_port))
                for port in routingTable[local_port]:
                    if port == local_port:
                        continue
                    if port != routingTable[local_port][port][0]:
                        print('- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                              routingTable[local_port][port][0]))
                    else:
                        print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

                # If there is any change in the distance vector, a node should send the updated information to its neighbors.
                # print(update,1)
                if update or sendingTime == 0:
                    sendingTime += 1
                    for port in graph.keys():
                        if port != local_port:
                            data = pickle.dumps(routingTable)
                            skt.sendto(data, ('localhost', port))
                            if flag == 1:
                                print('[{}] Link value message sent from Node {} to Node {}'.format(time.time(),
                                                                                                    local_port,
                                                                                                    port))
                            else:
                                print('[{}] Message sent from Node {} to Node {}'.format(time.time(), local_port, port))
            n += 1

def listendv(routingTable, cost):
    graph = cost
    sendingTime = 0
    while True:
        msg, addr = skt.recvfrom(2048)
        temp = pickle.loads(msg)
        #print(temp)
        try:
            graph[addr[1]] = temp[2]
            continue
        except:
            newTable = temp[addr[1]]
        flag = 0
        #print(newTable)
        if newTable[addr[1]][0] == 'flag':
            #newTable[addr[1]] = [addr[1], 0]
            flag = 1
            #print('test')
            if addr[1] != local_port:
                print('[{}] Link value message received at Node {} from Node {}'.format(time.time(), local_port, addr[1]))
        else: print('[{}] Message received at Node {} from Node {}'.format(time.time(), local_port, addr[1]))
        # print(pickle.loads(msg), addr)
        update = False
        # In every table: change -> only the line which represent the port changed. every other lines changed not count.
        # put new line into routing table

        # whether there are new nodes
        for port in newTable.keys():
            if port not in routingTable[local_port]:
                routingTable[port] = {}
                for node in routingTable.keys():
                    routingTable[node][port] = [None, float('inf')]
                    routingTable[port][node] = [None, float('inf')]
                update = True
                # print(update)

        # routingTable[addr[1]] = newTable
        for port in newTable:
            if routingTable[addr[1]][port] != newTable[port]:
                routingTable[addr[1]][port] = newTable[port]
                update = True

        #print(routingTable)

        # update
        for node in routingTable[local_port].keys():
            if node != local_port:
                routingTable[local_port][node][1] = float('inf')
                for port in graph:
                    if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                            routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                            routingTable[local_port][node][0] = port
                            #print(port)
                            #update = True

        # print table
        print('[{}] Node {} Routing Table'.format(time.time(), local_port))
        for port in routingTable[local_port]:
            if port == local_port:
                continue
            if port != routingTable[local_port][port][0]:
                print('- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                      routingTable[local_port][port][0]))
            else:
                print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

        # If there is any change in the distance vector, a node should send the updated information to its neighbors.
        # print(update,1)
        if update or sendingTime == 0:
            sendingTime += 1
            for port in graph.keys():
                if port != local_port:
                    data = pickle.dumps(routingTable)
                    skt.sendto(data, ('localhost', port))
                    if flag == 1:
                        print('[{}] Link value message sent from Node {} to Node {}'.format(time.time(), local_port, port))
                    else: print('[{}] Message sent from Node {} to Node {}'.format(time.time(), local_port, port))


if __name__ == '__main__':
    programe = sys.argv[1]
    mode = sys.argv[2]
    interval = sys.argv[3]
    local_port = int(sys.argv[4])
    if local_port <= 1024:
        print('Invalid local-port number')
        exit()
    if programe == 'dv':
        if mode == 'r':
            graph = {}
            lastNode = False
            graph[local_port] = 0
            i = 5
            while i < len(sys.argv):
                if sys.argv[i] == 'last':
                    lastNode = True
                    i += 1
                    costChange = 0
                    if i < len(sys.argv):
                        costChange = int(sys.argv[i])
                    break
                if int(sys.argv[i]) <= 1024:
                    print('Invalid local-port number' + sys.argv[i])
                    exit()
                graph[int(sys.argv[i])] = int(sys.argv[i + 1])
                i += 2
            # print(graph)
            routingTable = {}
            # Init the routing table
            for port in graph.keys():
                routingTable[port] = {}
                for destination in graph.keys():
                    if port == local_port:
                        routingTable[port][destination] = [destination, graph[destination]]
                    else:
                        routingTable[port][destination] = [None, float('inf')]
            # print(routingTable)

            # Print the routing table
            print('[{}] Node {} Routing Table'.format(time.time(), local_port))
            for port in routingTable[local_port]:
                if port == local_port:
                    continue
                if port != routingTable[local_port][port][0]:
                    print('- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                          routingTable[local_port][port][0]))
                else:
                    print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

            # Start sending
            skt = socket(AF_INET, SOCK_DGRAM)
            skt.bind(("", local_port))
            if lastNode:
                for port in routingTable.keys():
                    if port != local_port:
                        data = pickle.dumps(routingTable)
                        skt.sendto(data, ('localhost', port))
                        print(
                            '[{}] Message sent from Node <port-{}> to Node <port-{}>'.format(time.time(), local_port,
                                                                                             port))

            th = threading.Thread(target=listendv, args=(routingTable, graph))
            th.start()
            # listen(routingTable, graph)
            delay = 30
            close_time = time.time() + delay
            while True:
                if time.time() > close_time:
                    break
            if lastNode:
                maxi = 1024
                for port in graph:
                    if port != local_port:
                        maxi = max(port, maxi)
                graph[maxi] = costChange
                msg = [local_port, maxi, costChange]
                gra_change = pickle.dumps(msg)
                skt.sendto(gra_change, ('localhost', maxi))
                # print(routingTable)
                for node in routingTable[local_port].keys():
                    if node != local_port:
                        routingTable[local_port][node][1] = float('inf')
                        for port in graph:
                            if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                                routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                                routingTable[local_port][node][0] = port
                                # print(port)
                                update = True
                routingTable[local_port][local_port][0] = 'flag'
                data = pickle.dumps(routingTable)
                for port in graph:
                    if port != local_port:
                        skt.sendto(data, ('localhost', port))
                        print('[{}] Link value message sent from Node {} to Node {}'.format(time.time(), local_port,
                                                                                            port))
                print('[{}] Node <port-{}> cost updated to <{}>'.format(time.time(), maxi, costChange))
        elif mode == 'p':
            #print(1)
            graph = {}
            lastNode = False
            graph[local_port] = 0
            i = 5
            while i < len(sys.argv):
                if sys.argv[i] == 'last':
                    lastNode = True
                    i += 1
                    costChange = 0
                    if i < len(sys.argv):
                        costChange = int(sys.argv[i])
                    break
                if int(sys.argv[i]) <= 1024:
                    print('Invalid local-port number' + sys.argv[i])
                    exit()
                graph[int(sys.argv[i])] = int(sys.argv[i + 1])
                i+=2
            routingTable = {}
            # Init the routing table
            for port in graph.keys():
                routingTable[port] = {}
                for destination in graph.keys():
                    if port == local_port:
                        routingTable[port][destination] = [destination, graph[destination]]
                    else:
                        routingTable[port][destination] = [None, float('inf')]
            #print(routingTable)

            # Print the routing table
            print('[{}] Node {} Routing Table'.format(time.time(), local_port))
            for port in routingTable[local_port]:
                if port == local_port:
                    continue
                if port != routingTable[local_port][port][0]:
                    print('- ({}) -> Node {}; Next hop -> Node {}'.format(routingTable[local_port][port][1], port,
                                                                          routingTable[local_port][port][0]))
                else:
                    print('- ({}) -> Node {}'.format(routingTable[local_port][port][1], port))

            # Start sending
            skt = socket(AF_INET, SOCK_DGRAM)
            skt.bind(("", local_port))
            if lastNode:
                for port in routingTable.keys():
                    if port != local_port:
                        data = pickle.dumps(routingTable)
                        skt.sendto(data, ('localhost', port))
                        print('[{}] Message sent from Node <port-{}> to Node <port-{}>'.format(time.time(), local_port,
                                                                                               port))

            th = threading.Thread(target=listenp, args=(routingTable, graph))
            th.start()
            # listen(routingTable, graph)

            delay = 5
            close_time = time.time() + delay
            while True:
                if time.time() > close_time:
                    break
            if lastNode:
                msg = [0, 0, 0, 'poison']
                for node in routingTable[local_port].keys():
                    skt.sendto(pickle.dumps(msg), ('localhost', node))
                maxi = 1024
                for port in graph:
                    if port != local_port:
                        maxi = max(port, maxi)
                graph[maxi] = costChange
                msg = [local_port, maxi, costChange]
                gra_change = pickle.dumps(msg)
                skt.sendto(gra_change, ('localhost', maxi))
                # print(routingTable)
                temp = {}
                for node in routingTable[local_port].keys():
                    if node != local_port:
                        routingTable[local_port][node][1] = float('inf')
                        for port in graph:
                            if routingTable[local_port][node][1] > graph[port] + routingTable[port][node][1]:
                                routingTable[local_port][node][1] = graph[port] + routingTable[port][node][1]
                                routingTable[local_port][node][0] = port
                                if port != node:
                                    temp[port] = node
                                update = True
                # routingTable[local_port][local_port][0] = 'flag'

                for port in graph:
                    if port != local_port:
                        table = routingTable
                        if port in temp:
                            table[local_port][temp[port]] = [port, float('inf')]
                        data = pickle.dumps(table)

                        skt.sendto(data, ('localhost', port))
                        print('[{}] Link value message sent from Node {} to Node {}'.format(time.time(), local_port,
                                                                                            port))
                print('[{}] Node <port-{}> cost updated to <{}>'.format(time.time(), maxi, costChange))

    if programe == 'ls':
        other_port = {}
        graph = {}
        lastNode = False
        # graph[local_port] = 0
        i = 5
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
        # print(lst)
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


