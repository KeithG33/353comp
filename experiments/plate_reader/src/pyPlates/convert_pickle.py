from sys import argv
import pickle
import gzip


if __name__ == '__main__':
    print(argv)
    for filename in argv[1:]:
        print("Coverting: {}".format(filename))
        # with open(filename, 'rb') as f:
        #     pick = pickle.load(f)
        pick = pickle.load(open(filename, 'rb'))
        
        pickle.dump(pick, open("py2"+filename,"wb"), protocol=2)