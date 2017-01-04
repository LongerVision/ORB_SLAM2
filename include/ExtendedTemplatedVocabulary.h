#ifndef __D_T_EXTENDED_TEMPLATED_VOCABULARY__
#define __D_T_EXTENDED_TEMPLATED_VOCABULARY__

#include "DBoW2/TemplatedVocabulary.h"

namespace DBoW2 {
/// @param TDescriptor class of descriptor
/// @param F class of descriptor functions
template<class TDescriptor, class F>
/// Generic Vocabulary
class ExtendedTemplatedVocabulary : public TemplatedVocabulary<TDescriptor,F>
{
public:
  /**
   * Initiates an empty vocabulary
   * @param k branching factor
   * @param L depth levels
   * @param weighting weighting type
   * @param scoring scoring type
   */
  ExtendedTemplatedVocabulary(int k = 10, int L = 5, 
    WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);
  
  /**
   * Creates the vocabulary by loading a file
   * @param filename
   */
  ExtendedTemplatedVocabulary(const std::string &filename);
  
  /**
   * Creates the vocabulary by loading a file
   * @param filename
   */
  ExtendedTemplatedVocabulary(const char *filename);
  
  /** 
   * Copy constructor
   * @param voc
   */
  ExtendedTemplatedVocabulary(const ExtendedTemplatedVocabulary<TDescriptor, F> &voc);
  
  /**
   * Destructor
   */
  virtual ~ExtendedTemplatedVocabulary() {}
  
  /** 
   * Assigns the given vocabulary to this by copying its data and removing
   * all the data contained by this vocabulary before
   * @param voc
   * @return reference to this vocabulary
   */
  ExtendedTemplatedVocabulary<TDescriptor, F>& operator=(
    const ExtendedTemplatedVocabulary<TDescriptor, F> &voc)
  {
     this->m_k = voc.m_k;
      this->m_L = voc.m_L;
      this->m_scoring = voc.m_scoring;
      this->m_weighting = voc.m_weighting;

      this->createScoringObject();
      
      this->m_nodes.clear();
      this->m_words.clear();
      
      this->m_nodes = voc.m_nodes;
      this->createWords();
      
      return *this;
  }

  
  /**
   * Loads the vocabulary from a text file
   * @param filename
   */
  bool loadFromTextFile(const std::string &filename);

  /**
   * Saves the vocabulary into a text file
   * @param filename
   */
  void saveToTextFile(const std::string &filename) const;  

};


template<class TDescriptor, class F>
ExtendedTemplatedVocabulary<TDescriptor,F>::ExtendedTemplatedVocabulary(int k, int L, WeightingType weighting, ScoringType scoring)
: TemplatedVocabulary<TDescriptor,F>(k, L, weighting, scoring)
{
    
}

template<class TDescriptor, class F>
ExtendedTemplatedVocabulary<TDescriptor,F>::ExtendedTemplatedVocabulary(const std::string &filename)
: TemplatedVocabulary<TDescriptor,F>(filename)
{
    
}

template<class TDescriptor, class F>
ExtendedTemplatedVocabulary<TDescriptor,F>::ExtendedTemplatedVocabulary(const char *filename)
: TemplatedVocabulary<TDescriptor,F>(filename)
{
    
}

template<class TDescriptor, class F>
ExtendedTemplatedVocabulary<TDescriptor,F>::ExtendedTemplatedVocabulary(const ExtendedTemplatedVocabulary<TDescriptor, F> &voc)
: TemplatedVocabulary<TDescriptor,F>(voc)
{
    
}


template<class TDescriptor, class F>
bool ExtendedTemplatedVocabulary<TDescriptor,F>::loadFromTextFile(const std::string &filename)
{
    std::ifstream f;
    f.open(filename.c_str());
    
    if(f.eof())
        return false;

    this->m_words.clear();
    this->m_nodes.clear();

    std::string s;
    getline(f,s);
    std::stringstream ss;
    ss << s;
    ss >> this->m_k;
    ss >> this->m_L;
    int n1, n2;
    ss >> n1;
    ss >> n2;

    if(this->m_k<0 || this->m_k>20 || this->m_L<1 || this->m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
    {
        std::cerr << "Vocabulary loading failure: This is not a correct text file!" << std::endl;
        return false;
    }
    
    this->m_scoring = (ScoringType)n1;
    this->m_weighting = (WeightingType)n2;
    this->createScoringObject();

    // nodes
    int expected_nodes =
    (int)((pow((double)this->m_k, (double)this->m_L + 1) - 1)/(this->m_k - 1));
    this->m_nodes.reserve(expected_nodes);

    this->m_words.reserve(pow((double)this->m_k, (double)this->m_L + 1));

    this->m_nodes.resize(1);
    this->m_nodes[0].id = 0;
    while(!f.eof())
    {
        std::string snode;
        getline(f,snode);
        std::stringstream ssnode;
        ssnode << snode;

        int nid = this->m_nodes.size();
        this->m_nodes.resize(this->m_nodes.size()+1);
        this->m_nodes[nid].id = nid;
    
        int pid ;
        ssnode >> pid;
        this->m_nodes[nid].parent = pid;
        this->m_nodes[pid].children.push_back(nid);

        int nIsLeaf;
        ssnode >> nIsLeaf;

        std::stringstream ssd;
        for(int iD=0;iD<F::L;iD++)
        {
            std::string sElement;
            ssnode >> sElement;
            ssd << sElement << " ";
        }
        F::fromString(this->m_nodes[nid].descriptor, ssd.str());

        ssnode >> this->m_nodes[nid].weight;

        if(nIsLeaf>0)
        {
            int wid = this->m_words.size();
            this->m_words.resize(wid+1);

            this->m_nodes[nid].word_id = wid;
            this->m_words[wid] = &this->m_nodes[nid];
        }
        else
        {
            this->m_nodes[nid].children.reserve(this->m_k);
        }
    }

    return true;

}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void ExtendedTemplatedVocabulary<TDescriptor,F>::saveToTextFile(const std::string &filename) const
{
    std::fstream f;
    f.open(filename.c_str(),std::ios_base::out);
    f << this->m_k << " " << this->m_L << " " << " " << this->m_scoring << " " << this->m_weighting << std::endl;

    for(size_t i=1; i<this->m_nodes.size();i++)
    {
        f << this->m_nodes[i].parent << " ";
        if(this->m_nodes[i].isLeaf())
            f << 1 << " ";
        else
            f << 0 << " ";

        f << F::toString(this->m_nodes[i].descriptor) << " " << (double)this->m_nodes[i].weight << std::endl;
    }

    f.close();
}


} // namespace DBoW2


#endif    // __D_T_EXTENDED_TEMPLATED_VOCABULARY__

