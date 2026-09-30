"""Altered Python translation of Dagor's nonbranchless pdqsort, for radar rows.

Source: dagor-source/dag_stlqsort.h, pinned in sort-source-manifest.json.
Compared to native 143ff0d30, aces.exe 2.59.0.34. Heap movement/tie handling
follows the native inlined EASTL fallback. Inputs are finite float32 triples;
comparison uses only row[2]. Equal scores need not preserve enumeration order.

Original pdqsort notice (retained for this altered source):

Copyright (c) 2015 Orson Peters

This software is provided 'as-is', without any express or implied warranty. In no event will the
authors be held liable for any damages arising from the use of this software.

Permission is granted to anyone to use this software for any purpose, including commercial
applications, and to alter it and redistribute it freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the
   original software. If you use this software in a product, an acknowledgment in the product
   documentation would be appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not be misrepresented as
   being the original software.

3. This notice may not be removed or altered from any source distribution.

https://github.com/orlp/pdqsort
"""
import math
from kernels import f32


def order_candidates(rows,*,bad_allowed=None,events=None):
    """Return copied rows in native order; optional budget/events are probe aids."""
    a=[list(map(f32,row)) for row in rows]
    if any(len(row)!=3 or not all(math.isfinite(x) for x in row) for row in a):
        raise ValueError('Candidate ordering requires finite float32 triples')
    if not a:return a
    if bad_allowed is None:bad_allowed=len(a).bit_length()-1
    if len(a)>=24 and bad_allowed<1:raise ValueError('Partition budget must be positive')
    def mark(name):
        if events is not None:events[name]=events.get(name,0)+1
    def swap(i,j):a[i],a[j]=a[j],a[i]
    def less(i,j):return a[i][2]<a[j][2]
    def sort3(i,j,k):
        if less(j,i):swap(i,j)
        if less(k,j):swap(j,k)
        if less(j,i):swap(i,j)
    def insertion(begin,end,partial=False):
        moves=0
        for cur in range(begin+1,end):
            if partial and moves>8:
                mark('partial_abort');return False
            if less(cur,cur-1):
                value=a[cur];hole=cur
                while hole>begin and value[2]<a[hole-1][2]:
                    a[hole]=a[hole-1];hole-=1
                a[hole]=value;moves+=cur-hole
        return True
    def adjust_heap(begin,hole,length,value):
        top=hole;right=2*hole+2
        while right<length:
            child=right-1 if a[begin+right][2]<a[begin+right-1][2] else right
            a[begin+hole]=a[begin+child];hole=child;right=2*hole+2
        if right==length:
            a[begin+hole]=a[begin+right-1];hole=right-1
        while hole>top:
            parent=(hole-1)//2
            if value[2]<=a[begin+parent][2]:break
            a[begin+hole]=a[begin+parent];hole=parent
        a[begin+hole]=value
    def heap_sort(begin,end):
        mark('heap');length=end-begin
        for hole in range((length-2)//2,-1,-1):adjust_heap(begin,hole,length,a[begin+hole])
        while length>1:
            value=a[begin+length-1];a[begin+length-1]=a[begin];length-=1
            adjust_heap(begin,0,length,value)
    def partition_right(begin,end):
        pivot=a[begin];first=begin+1;last=end
        while a[first][2]<pivot[2]:first+=1
        if first-1==begin:
            while first<last:
                last-=1
                if a[last][2]<pivot[2]:break
        else:
            last-=1
            while not a[last][2]<pivot[2]:last-=1
        already=first>=last
        while first<last:
            swap(first,last);first+=1;last-=1
            while a[first][2]<pivot[2]:first+=1
            while not a[last][2]<pivot[2]:last-=1
        pos=first-1;a[begin]=a[pos];a[pos]=pivot
        return pos,already
    def partition_left(begin,end):
        mark('partition_left');pivot=a[begin];first=begin;last=end-1
        while pivot[2]<a[last][2]:last-=1
        if last+1==end:
            while first<last:
                first+=1
                if pivot[2]<a[first][2]:break
        else:
            first+=1
            while not pivot[2]<a[first][2]:first+=1
        while first<last:
            swap(first,last);last-=1;first+=1
            while pivot[2]<a[last][2]:last-=1
            while not pivot[2]<a[first][2]:first+=1
        a[begin]=a[last];a[last]=pivot
        return last
    def loop(begin,end,budget,leftmost=True):
        while True:
            size=end-begin
            if size<24:
                mark('insertion');insertion(begin,end);return
            mid=begin+size//2
            if size>128:
                mark('ninther');sort3(begin,mid,end-1);sort3(begin+1,mid-1,end-2)
                sort3(begin+2,mid+1,end-3);sort3(mid-1,mid,mid+1);swap(begin,mid)
            else:mark('median3');sort3(mid,begin,end-1)
            if not leftmost and not less(begin-1,begin):
                begin=partition_left(begin,end)+1;continue
            pivot,already=partition_right(begin,end);left=pivot-begin;right=end-pivot-1
            if left<size//8 or right<size//8:
                mark('unbalanced');budget-=1
                if budget==0:heap_sort(begin,end);return
                if left>=24:
                    swap(begin,begin+left//4);swap(pivot-1,pivot-left//4)
                    if left>128:
                        swap(begin+1,begin+left//4+1);swap(begin+2,begin+left//4+2)
                        swap(pivot-2,pivot-left//4-1);swap(pivot-3,pivot-left//4-2)
                if right>=24:
                    swap(pivot+1,pivot+1+right//4);swap(end-1,end-right//4)
                    if right>128:
                        swap(pivot+2,pivot+2+right//4);swap(pivot+3,pivot+3+right//4)
                        swap(end-2,end-1-right//4);swap(end-3,end-2-right//4)
            elif already:
                mark('partial')
                if insertion(begin,pivot,True) and insertion(pivot+1,end,True):return
            loop(begin,pivot,budget,leftmost);begin=pivot+1;leftmost=False
    loop(0,len(a),bad_allowed)
    return a
