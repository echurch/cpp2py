//
// File generated by rootcint at Sun Sep 14 14:34:07 2014

// Do NOT change. Changes will be lost next time file is generated
//

#define R__DICTIONARY_FILENAME DaqFileCint
#include "RConfig.h" //rootcint 4834
#if !defined(R__ACCESS_IN_SYMBOL)
//Break the privacy of classes -- Disabled for the moment
#define private public
#define protected public
#endif

// Since CINT ignores the std namespace, we need to do so in this file.
namespace std {} using namespace std;
#include "DaqFileCint.h"

#include "TClass.h"
#include "TBuffer.h"
#include "TMemberInspector.h"
#include "TError.h"

#ifndef G__ROOT
#define G__ROOT
#endif

#include "RtypesImp.h"
#include "TIsAProxy.h"
#include "TFileMergeInfo.h"

// START OF SHADOWS

namespace ROOT {
   namespace Shadow {
      #if !(defined(R__ACCESS_IN_SYMBOL) || defined(R__USE_SHADOW_CLASS))
      typedef ::DaqFile DaqFile;
      #else
      class DaqFile  {
         public:
         //friend XX;
         // To force the creation of a virtual table, throw just in case.
         virtual ~DaqFile() throw() {};
         bool good; //
         bool closedCleanly; //
         unsigned int m_entry; //
         unsigned int nevents; //
         unsigned int* index_buffer; //
         ::basic_ifstream< char, ::char_traits< char >  > ifs; //
      };
      #endif

   } // of namespace Shadow
} // of namespace ROOT
// END OF SHADOWS

namespace ROOT {
   void DaqFile_ShowMembers(void *obj, TMemberInspector &R__insp);
   static void DaqFile_Dictionary();
   static void *new_DaqFile(void *p = 0);

   // Function generating the singleton type initializer
   static TGenericClassInfo *GenerateInitInstanceLocal(const ::DaqFile*)
   {
      // Make sure the shadow class has the right sizeof
      R__ASSERT(sizeof(::DaqFile) == sizeof(::ROOT::Shadow::DaqFile));
      ::DaqFile *ptr = 0;
      static ::TVirtualIsAProxy* isa_proxy = new ::TIsAProxy(typeid(::DaqFile),0);
      static ::ROOT::TGenericClassInfo 
         instance("DaqFile", "./DaqFile.hh", 34,
                  typeid(::DaqFile), DefineBehavior(ptr, ptr),
                  &DaqFile_ShowMembers, &DaqFile_Dictionary, isa_proxy, 4,
                  sizeof(::DaqFile) );
      instance.SetNew(&new_DaqFile);
      return &instance;
   }
   TGenericClassInfo *GenerateInitInstance(const ::DaqFile*)
   {
      return GenerateInitInstanceLocal((::DaqFile*)0);
   }
   // Static variable to force the class initialization
   static ::ROOT::TGenericClassInfo *_R__UNIQUE_(Init) = GenerateInitInstanceLocal((const ::DaqFile*)0x0); R__UseDummy(_R__UNIQUE_(Init));

   // Dictionary for non-ClassDef classes
   static void DaqFile_Dictionary() {
      ::ROOT::GenerateInitInstanceLocal((const ::DaqFile*)0x0)->GetClass();
   }

} // end of namespace ROOT

//______________________________________________________________________________
namespace ROOT {
   void DaqFile_ShowMembers(void *obj, TMemberInspector &R__insp)
   {
      // Inspect the data members of an object of class DaqFile.
      typedef ::ROOT::Shadow::DaqFile ShadowClass;
      ShadowClass *sobj = (ShadowClass*)obj;
      if (sobj) { } // Dummy usage just in case there is no datamember.

      TClass *R__cl  = ::ROOT::GenerateInitInstanceLocal((const ::DaqFile*)0x0)->GetClass();
      if (R__cl || R__insp.IsA()) { }
      R__insp.Inspect(R__cl, R__insp.GetParent(), "good", &sobj->good);
      R__insp.Inspect(R__cl, R__insp.GetParent(), "closedCleanly", &sobj->closedCleanly);
      R__insp.Inspect(R__cl, R__insp.GetParent(), "m_entry", &sobj->m_entry);
      R__insp.Inspect(R__cl, R__insp.GetParent(), "nevents", &sobj->nevents);
      R__insp.Inspect(R__cl, R__insp.GetParent(), "*index_buffer", &sobj->index_buffer);
      R__insp.Inspect(R__cl, R__insp.GetParent(), "ifs", (void*)&sobj->ifs);
      R__insp.InspectMember("ifstream", (void*)&sobj->ifs, "ifs.", false);
   }

}

namespace ROOT {
   // Wrappers around operator new
   static void *new_DaqFile(void *p) {
      return  p ? ::new((::ROOT::TOperatorNewHelper*)p) ::DaqFile : new ::DaqFile;
   }
} // end of namespace ROOT for class ::DaqFile

/********************************************************
* DaqFileCint.cc
* CAUTION: DON'T CHANGE THIS FILE. THIS FILE IS AUTOMATICALLY GENERATED
*          FROM HEADER FILES LISTED IN G__setup_cpp_environmentXXX().
*          CHANGE THOSE HEADER FILES AND REGENERATE THIS FILE.
********************************************************/

#ifdef G__MEMTEST
#undef malloc
#undef free
#endif

#if defined(__GNUC__) && __GNUC__ >= 4 && ((__GNUC_MINOR__ == 2 && __GNUC_PATCHLEVEL__ >= 1) || (__GNUC_MINOR__ >= 3))
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif

extern "C" void G__cpp_reset_tagtableDaqFileCint();

extern "C" void G__set_cpp_environmentDaqFileCint() {
  G__add_compiledheader("TObject.h");
  G__add_compiledheader("TMemberInspector.h");
  G__add_compiledheader("DaqFile.hh");
  G__add_compiledheader("DaqFile-TypeDef.hh");
  G__cpp_reset_tagtableDaqFileCint();
}
#include <new>
extern "C" int G__cpp_dllrevDaqFileCint() { return(30051515); }

/*********************************************************
* Member function Interface Method
*********************************************************/

/* DaqFile */
static int G__DaqFileCint_189_0_1(G__value* result7, G__CONST char* funcname, struct G__param* libp, int hash)
{
   DaqFile* p = NULL;
   char* gvp = (char*) G__getgvp();
   int n = G__getaryconstruct();
   if (n) {
     p = 0;
     G__genericerror("Error: Array construction with private/protected destructor is illegal");
   } else {
     if ((gvp == (char*)G__PVOID) || (gvp == 0)) {
       p = new DaqFile;
     } else {
       p = new((void*) gvp) DaqFile;
     }
   }
   result7->obj.i = (long) p;
   result7->ref = (long) p;
   G__set_tagnum(result7,G__get_linked_tagnum(&G__DaqFileCintLN_DaqFile));
   return(1 || funcname || hash || result7 || libp) ;
}

static int G__DaqFileCint_189_0_2(G__value* result7, G__CONST char* funcname, struct G__param* libp, int hash)
{
   DaqFile* p = NULL;
   char* gvp = (char*) G__getgvp();
   //m: 1
   if ((gvp == (char*)G__PVOID) || (gvp == 0)) {
     p = new DaqFile(*(string*) libp->para[0].ref);
   } else {
     p = new((void*) gvp) DaqFile(*(string*) libp->para[0].ref);
   }
   result7->obj.i = (long) p;
   result7->ref = (long) p;
   G__set_tagnum(result7,G__get_linked_tagnum(&G__DaqFileCintLN_DaqFile));
   return(1 || funcname || hash || result7 || libp) ;
}

static int G__DaqFileCint_189_0_3(G__value* result7, G__CONST char* funcname, struct G__param* libp, int hash)
{
      G__letint(result7, 103, (long) ((DaqFile*) G__getstructoffset())->Good());
   return(1 || funcname || hash || result7 || libp) ;
}

static int G__DaqFileCint_189_0_4(G__value* result7, G__CONST char* funcname, struct G__param* libp, int hash)
{
      G__letint(result7, 103, (long) ((DaqFile*) G__getstructoffset())->ClosedCleanly());
   return(1 || funcname || hash || result7 || libp) ;
}

static int G__DaqFileCint_189_0_5(G__value* result7, G__CONST char* funcname, struct G__param* libp, int hash)
{
      G__letint(result7, 105, (long) ((DaqFile*) G__getstructoffset())->NumEvents());
   return(1 || funcname || hash || result7 || libp) ;
}

static int G__DaqFileCint_189_0_6(G__value* result7, G__CONST char* funcname, struct G__param* libp, int hash)
{
      G__letint(result7, 105, (long) ((DaqFile*) G__getstructoffset())->GetEventData((unsigned int) G__int(libp->para[0]), libp->para[1].ref ? *(char**) libp->para[1].ref : *(char**) (void*) (&G__Mlong(libp->para[1]))
, *(size_t*) G__ULongref(&libp->para[2])));
   return(1 || funcname || hash || result7 || libp) ;
}


/* Setting up global function */

/*********************************************************
* Member function Stub
*********************************************************/

/* DaqFile */

/*********************************************************
* Global function Stub
*********************************************************/

/*********************************************************
* Get size of pointer to member function
*********************************************************/
class G__Sizep2memfuncDaqFileCint {
 public:
  G__Sizep2memfuncDaqFileCint(): p(&G__Sizep2memfuncDaqFileCint::sizep2memfunc) {}
    size_t sizep2memfunc() { return(sizeof(p)); }
  private:
    size_t (G__Sizep2memfuncDaqFileCint::*p)();
};

size_t G__get_sizep2memfuncDaqFileCint()
{
  G__Sizep2memfuncDaqFileCint a;
  G__setsizep2memfunc((int)a.sizep2memfunc());
  return((size_t)a.sizep2memfunc());
}


/*********************************************************
* virtual base class offset calculation interface
*********************************************************/

   /* Setting up class inheritance */

/*********************************************************
* Inheritance information setup/
*********************************************************/
extern "C" void G__cpp_setup_inheritanceDaqFileCint() {

   /* Setting up class inheritance */
}

/*********************************************************
* typedef information setup/
*********************************************************/
extern "C" void G__cpp_setup_typetableDaqFileCint() {

   /* Setting up typedef entry */
   G__search_typename2("vector<ROOT::TSchemaHelper>",117,G__get_linked_tagnum(&G__DaqFileCintLN_vectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgR),0,-1);
   G__setnewtype(-1,NULL,0);
   G__search_typename2("reverse_iterator<const_iterator>",117,G__get_linked_tagnum(&G__DaqFileCintLN_reverse_iteratorlEvectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgRcLcLiteratorgR),0,G__get_linked_tagnum(&G__DaqFileCintLN_vectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgR));
   G__setnewtype(-1,NULL,0);
   G__search_typename2("reverse_iterator<iterator>",117,G__get_linked_tagnum(&G__DaqFileCintLN_reverse_iteratorlEvectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgRcLcLiteratorgR),0,G__get_linked_tagnum(&G__DaqFileCintLN_vectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgR));
   G__setnewtype(-1,NULL,0);
   G__search_typename2("vector<TVirtualArray*>",117,G__get_linked_tagnum(&G__DaqFileCintLN_vectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgR),0,-1);
   G__setnewtype(-1,NULL,0);
   G__search_typename2("reverse_iterator<const_iterator>",117,G__get_linked_tagnum(&G__DaqFileCintLN_reverse_iteratorlEvectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgRcLcLiteratorgR),0,G__get_linked_tagnum(&G__DaqFileCintLN_vectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgR));
   G__setnewtype(-1,NULL,0);
   G__search_typename2("reverse_iterator<iterator>",117,G__get_linked_tagnum(&G__DaqFileCintLN_reverse_iteratorlEvectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgRcLcLiteratorgR),0,G__get_linked_tagnum(&G__DaqFileCintLN_vectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgR));
   G__setnewtype(-1,NULL,0);
}

/*********************************************************
* Data Member information setup/
*********************************************************/

   /* Setting up class,struct,union tag member variable */

   /* DaqFile */
static void G__setup_memvarDaqFile(void) {
   G__tag_memvar_setup(G__get_linked_tagnum(&G__DaqFileCintLN_DaqFile));
   { DaqFile *p; p=(DaqFile*)0x1000; if (p) { }
   G__memvar_setup((void*)0,103,0,0,-1,-1,-1,4,"good=",0,(char*)NULL);
   G__memvar_setup((void*)0,103,0,0,-1,-1,-1,4,"closedCleanly=",0,(char*)NULL);
   G__memvar_setup((void*)0,104,0,0,-1,G__defined_typename("uint32_t"),-1,4,"m_entry=",0,(char*)NULL);
   G__memvar_setup((void*)0,104,0,0,-1,G__defined_typename("uint32_t"),-1,4,"nevents=",0,(char*)NULL);
   G__memvar_setup((void*)0,72,0,0,-1,G__defined_typename("uint32_t"),-1,4,"index_buffer=",0,(char*)NULL);
   G__memvar_setup((void*)0,117,0,0,G__get_linked_tagnum(&G__DaqFileCintLN_basic_ifstreamlEcharcOchar_traitslEchargRsPgR),G__defined_typename("ifstream"),-1,4,"ifs=",0,(char*)NULL);
   G__memvar_setup((void*)0,108,0,0,-1,-1,-1,4,"G__virtualinfo=",0,(char*)NULL);
   }
   G__tag_memvar_reset();
}

extern "C" void G__cpp_setup_memvarDaqFileCint() {
}
/***********************************************************
************************************************************
************************************************************
************************************************************
************************************************************
************************************************************
************************************************************
***********************************************************/

/*********************************************************
* Member function information setup for each class
*********************************************************/
static void G__setup_memfuncDaqFile(void) {
   /* DaqFile */
   G__tag_memfunc_setup(G__get_linked_tagnum(&G__DaqFileCintLN_DaqFile));
   G__memfunc_setup("DaqFile",662,G__DaqFileCint_189_0_1, 105, G__get_linked_tagnum(&G__DaqFileCintLN_DaqFile), -1, 0, 0, 1, 1, 0, "", (char*)NULL, (void*) NULL, 0);
   G__memfunc_setup("DaqFile",662,G__DaqFileCint_189_0_2, 105, G__get_linked_tagnum(&G__DaqFileCintLN_DaqFile), -1, 0, 1, 1, 1, 0, "u 'string' - 11 - pathname", (char*)NULL, (void*) NULL, 0);
   G__memfunc_setup("Good",393,G__DaqFileCint_189_0_3, 103, -1, -1, 0, 0, 1, 1, 0, "", (char*)NULL, (void*) NULL, 0);
   G__memfunc_setup("ClosedCleanly",1314,G__DaqFileCint_189_0_4, 103, -1, -1, 0, 0, 1, 1, 0, "", (char*)NULL, (void*) NULL, 0);
   G__memfunc_setup("NumEvents",933,G__DaqFileCint_189_0_5, 105, -1, -1, 0, 0, 1, 1, 0, "", "/ only valid if ClosedCleanly() ", (void*) NULL, 0);
   G__memfunc_setup("GetEventData",1180,G__DaqFileCint_189_0_6, 105, -1, -1, 0, 3, 1, 1, 0, 
"h - - 0 - entry C - - 1 - outEventData "
"k - 'size_t' 1 - outEventSize", (char*)NULL, (void*) NULL, 0);
   G__tag_memfunc_reset();
}


/*********************************************************
* Member function information setup
*********************************************************/
extern "C" void G__cpp_setup_memfuncDaqFileCint() {
}

/*********************************************************
* Global variable information setup for each class
*********************************************************/
static void G__cpp_setup_global0() {

   /* Setting up global variables */
   G__resetplocal();

}

static void G__cpp_setup_global1() {
}

static void G__cpp_setup_global2() {

   G__resetglobalenv();
}
extern "C" void G__cpp_setup_globalDaqFileCint() {
  G__cpp_setup_global0();
  G__cpp_setup_global1();
  G__cpp_setup_global2();
}

/*********************************************************
* Global function information setup for each class
*********************************************************/
static void G__cpp_setup_func0() {
   G__lastifuncposition();

}

static void G__cpp_setup_func1() {
}

static void G__cpp_setup_func2() {
}

static void G__cpp_setup_func3() {
}

static void G__cpp_setup_func4() {
}

static void G__cpp_setup_func5() {
}

static void G__cpp_setup_func6() {
}

static void G__cpp_setup_func7() {
}

static void G__cpp_setup_func8() {
}

static void G__cpp_setup_func9() {
}

static void G__cpp_setup_func10() {
}

static void G__cpp_setup_func11() {
}

static void G__cpp_setup_func12() {

   G__resetifuncposition();
}

extern "C" void G__cpp_setup_funcDaqFileCint() {
  G__cpp_setup_func0();
  G__cpp_setup_func1();
  G__cpp_setup_func2();
  G__cpp_setup_func3();
  G__cpp_setup_func4();
  G__cpp_setup_func5();
  G__cpp_setup_func6();
  G__cpp_setup_func7();
  G__cpp_setup_func8();
  G__cpp_setup_func9();
  G__cpp_setup_func10();
  G__cpp_setup_func11();
  G__cpp_setup_func12();
}

/*********************************************************
* Class,struct,union,enum tag information setup
*********************************************************/
/* Setup class/struct taginfo */
G__linked_taginfo G__DaqFileCintLN_basic_ifstreamlEcharcOchar_traitslEchargRsPgR = { "basic_ifstream<char,char_traits<char> >" , 99 , -1 };
G__linked_taginfo G__DaqFileCintLN_string = { "string" , 99 , -1 };
G__linked_taginfo G__DaqFileCintLN_vectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgR = { "vector<ROOT::TSchemaHelper,allocator<ROOT::TSchemaHelper> >" , 99 , -1 };
G__linked_taginfo G__DaqFileCintLN_reverse_iteratorlEvectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgRcLcLiteratorgR = { "reverse_iterator<vector<ROOT::TSchemaHelper,allocator<ROOT::TSchemaHelper> >::iterator>" , 99 , -1 };
G__linked_taginfo G__DaqFileCintLN_vectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgR = { "vector<TVirtualArray*,allocator<TVirtualArray*> >" , 99 , -1 };
G__linked_taginfo G__DaqFileCintLN_reverse_iteratorlEvectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgRcLcLiteratorgR = { "reverse_iterator<vector<TVirtualArray*,allocator<TVirtualArray*> >::iterator>" , 99 , -1 };
G__linked_taginfo G__DaqFileCintLN_DaqFile = { "DaqFile" , 99 , -1 };

/* Reset class/struct taginfo */
extern "C" void G__cpp_reset_tagtableDaqFileCint() {
  G__DaqFileCintLN_basic_ifstreamlEcharcOchar_traitslEchargRsPgR.tagnum = -1 ;
  G__DaqFileCintLN_string.tagnum = -1 ;
  G__DaqFileCintLN_vectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgR.tagnum = -1 ;
  G__DaqFileCintLN_reverse_iteratorlEvectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgRcLcLiteratorgR.tagnum = -1 ;
  G__DaqFileCintLN_vectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgR.tagnum = -1 ;
  G__DaqFileCintLN_reverse_iteratorlEvectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgRcLcLiteratorgR.tagnum = -1 ;
  G__DaqFileCintLN_DaqFile.tagnum = -1 ;
}


extern "C" void G__cpp_setup_tagtableDaqFileCint() {

   /* Setting up class,struct,union tag entry */
   G__get_linked_tagnum_fwd(&G__DaqFileCintLN_basic_ifstreamlEcharcOchar_traitslEchargRsPgR);
   G__get_linked_tagnum_fwd(&G__DaqFileCintLN_string);
   G__get_linked_tagnum_fwd(&G__DaqFileCintLN_vectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgR);
   G__get_linked_tagnum_fwd(&G__DaqFileCintLN_reverse_iteratorlEvectorlEROOTcLcLTSchemaHelpercOallocatorlEROOTcLcLTSchemaHelpergRsPgRcLcLiteratorgR);
   G__get_linked_tagnum_fwd(&G__DaqFileCintLN_vectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgR);
   G__get_linked_tagnum_fwd(&G__DaqFileCintLN_reverse_iteratorlEvectorlETVirtualArraymUcOallocatorlETVirtualArraymUgRsPgRcLcLiteratorgR);
   G__tagtable_setup(G__get_linked_tagnum_fwd(&G__DaqFileCintLN_DaqFile),sizeof(DaqFile),-1,296192,(char*)NULL,G__setup_memvarDaqFile,G__setup_memfuncDaqFile);
}
extern "C" void G__cpp_setupDaqFileCint(void) {
  G__check_setup_version(30051515,"G__cpp_setupDaqFileCint()");
  G__set_cpp_environmentDaqFileCint();
  G__cpp_setup_tagtableDaqFileCint();

  G__cpp_setup_inheritanceDaqFileCint();

  G__cpp_setup_typetableDaqFileCint();

  G__cpp_setup_memvarDaqFileCint();

  G__cpp_setup_memfuncDaqFileCint();
  G__cpp_setup_globalDaqFileCint();
  G__cpp_setup_funcDaqFileCint();

   if(0==G__getsizep2memfunc()) G__get_sizep2memfuncDaqFileCint();
  return;
}
class G__cpp_setup_initDaqFileCint {
  public:
    G__cpp_setup_initDaqFileCint() { G__add_setup_func("DaqFileCint",(G__incsetup)(&G__cpp_setupDaqFileCint)); G__call_setup_funcs(); }
   ~G__cpp_setup_initDaqFileCint() { G__remove_setup_func("DaqFileCint"); }
};
G__cpp_setup_initDaqFileCint G__cpp_setup_initializerDaqFileCint;

